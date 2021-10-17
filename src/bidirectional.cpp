#include <cv.hpp>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "rvl.hpp"
#include <topic_compression/CompressionMeta.h>
#include <topic_compression/CompressedImage.h>
#include <topic_compression/CompressedDepthImage.h>

#include <pybind11/pybind11.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

using namespace pybind11::literals;

bool is_ros_node = false;
ros::Publisher publisher;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::chrono::duration;
auto t1 = high_resolution_clock::now();
auto t2 = high_resolution_clock::now();

void not_yet_implemented(const std::string &what) {
    throw ros::Exception("Not Yet Implemented: " + what);
}

ros::master::TopicInfo get_topic_info(std::string topic) {
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    std::string wanted_topic = topic.erase(topic.find_last_not_of('/') + 1);
    for (auto &master_topic : master_topics) {
        if (master_topic.name == wanted_topic)
            return master_topic;
    }
    throw ros::Exception("Topic '" + topic + "' does not exist.");
}

topic_compression::CompressedDepthImage image_to_compressed_depth(const sensor_msgs::ImageConstPtr &depth_msg) {
    int U = depth_msg->width;
    int V = depth_msg->height;
    char *output = (char *) malloc(V * U * sizeof(uint16_t));

    auto cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    if ("16UC1" != depth_msg->encoding) {
        ROS_INFO("Encoding is not supported, it must be 16UC1");
    }

    int n = CompressRVL(cv_depth_ptr->image.ptr<uint16_t>(0), output, V * U);

    topic_compression::CompressedDepthImage rvl_msg;

    rvl_msg.header = std_msgs::Header();
    rvl_msg.meta.header = depth_msg->header;
    rvl_msg.meta.height = depth_msg->height;
    rvl_msg.meta.width = depth_msg->width;
    rvl_msg.meta.encoding = depth_msg->encoding;
    rvl_msg.meta.is_bigendian = depth_msg->is_bigendian;
    rvl_msg.meta.step = depth_msg->step;
    rvl_msg.meta.algorithm = "RVL (2017) from https://github.com/RaymondKirk/topic_compression";

    rvl_msg.data.clear();

    for (int i = 0; i < n; i++) {
        rvl_msg.data.push_back(output[i]);
    }

    if (is_ros_node) {
        publisher.publish(rvl_msg);
        t2 = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;
        auto hz = (1.0 / ms_double.count()) * 1000;
        t1 = high_resolution_clock::now();
        ROS_DEBUG_STREAM("Compression rate from depth to RVL depth @ " << hz << "hz");
    }

    free(output);
    return rvl_msg;
}


sensor_msgs::Image compressed_depth_to_image(const topic_compression::CompressedDepthImage::ConstPtr &cmp_msg) {
    int U = cmp_msg->meta.width;
    int V = cmp_msg->meta.height;

    auto *output = (uint16_t *) malloc(V * U * sizeof(uint16_t));
    char *dataMat = (char *) malloc(V * U * sizeof(uint16_t));

    int i = 0;
    for (signed char it : cmp_msg->data) {
        dataMat[i] = it;
        i++;
    }

    DecompressRVL((char *) dataMat, (uint16_t *) output, V * U);
    cv::Mat image(V, U, CV_16UC1, output);

    cv_bridge::CvImagePtr cv_depth_ptr(new cv_bridge::CvImage);

    cv_depth_ptr->encoding = "16UC1";
    cv_depth_ptr->header = cmp_msg->meta.header;
    cv_depth_ptr->image = image;

    auto img_msg_ptr = cv_depth_ptr->toImageMsg();
    sensor_msgs::Image img_msg = *img_msg_ptr;

    if (is_ros_node) {
        publisher.publish(img_msg);
        t2 = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;
        auto hz = (1.0 / ms_double.count()) * 1000;
        t1 = high_resolution_clock::now();
        ROS_DEBUG_STREAM("Decompression rate from RVL to depth @ " << hz << "hz");
    }

    free(output);
    free(dataMat);
    return img_msg;
}

topic_compression::CompressedImage image_to_compressed_colour(const sensor_msgs::Image::ConstPtr &image) {
    topic_compression::CompressedImage compressed;
    compressed.header = std_msgs::Header();

    compressed.meta.header = image->header;
    compressed.meta.height = image->height;
    compressed.meta.width = image->width;
    compressed.meta.encoding = image->encoding;
    compressed.meta.is_bigendian = image->is_bigendian;
    compressed.meta.step = image->step;
    compressed.meta.algorithm = "JPEG (opencv) from https://github.com/RaymondKirk/topic_compression";

    compressed.data.header = image->header;
    compressed.data.format = image->encoding;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, "rgb8");
    if (!cv::imencode(".jpg", cv_ptr->image, compressed.data.data))
        ROS_ERROR_STREAM("Failed to compress colour image with JPEG compression");

    if (is_ros_node) {
        publisher.publish(compressed);
        t2 = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;
        auto hz = (1.0 / ms_double.count()) * 1000;
        t1 = high_resolution_clock::now();
        ROS_DEBUG_STREAM("Compression rate from RGB to JPEG RGB @ " << hz << "hz");
    }
    return compressed;
}

sensor_msgs::Image compressed_colour_to_image(const topic_compression::CompressedImage::ConstPtr &image) {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr->header = image->meta.header;
    cv_ptr->image = cv::imdecode(cv::Mat(image->data.data), CV_LOAD_IMAGE_UNCHANGED);

    cv_ptr->encoding = image->meta.encoding;
    if (image->meta.encoding == "bgr8") {  // ensure original format is preserved
        cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);
    }

    auto decompressed_ptr = cv_ptr->toImageMsg();
    sensor_msgs::Image decompressed = *decompressed_ptr;

    if (is_ros_node) {
        publisher.publish(decompressed);

        t2 = high_resolution_clock::now();
        duration<double, std::milli> ms_double = t2 - t1;
        auto hz = (1.0 / ms_double.count()) * 1000;
        t1 = high_resolution_clock::now();
        ROS_DEBUG_STREAM("Decompression rate from JPEG RGB to RGB @ " << hz << "hz");
    }

    return decompressed;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "topic_compression", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    is_ros_node = true;
    // Get in topic and out topic names
    std::string in_topic = nh.resolveName("in");
    std::string out_topic = nh.resolveName("out");
    if (in_topic.empty() || in_topic == "/in") {
        printf("Usage: %s in:=<in_base_topic> out:=<out_base_topic>\n", argv[0]);
        return 0;
    }
    if (in_topic[in_topic.length()] != '/')
        in_topic += '/';

    ros::master::TopicInfo info = get_topic_info(in_topic);

    // TODO: Format this similar to image_transport with clear in and out transports for pub/subs
    // TODO: This is horrible make this dynamic
    if (info.datatype == "sensor_msgs/Image") {
        if (out_topic.empty() || out_topic == "/out")
            out_topic = in_topic + "compressed";
        auto msg_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(in_topic);
        if (msg_ptr->encoding == "16UC1") {
            ROS_INFO_STREAM("Compressing depth from '" << in_topic << "' to '" << out_topic << "'");
            publisher = nh.advertise<topic_compression::CompressedDepthImage>(out_topic, 30);
            image_transport::ImageTransport it(nh);
            image_transport::Subscriber subscriber = it.subscribe(in_topic, 1, image_to_compressed_depth);
            ros::spin();
        } else if (std::string("rgb8bgr88UC3").find(msg_ptr->encoding) != std::string::npos) {
            ROS_INFO_STREAM("Compressing colour from '" << in_topic << "' to '" << out_topic << "'");
            publisher = nh.advertise<topic_compression::CompressedImage>(out_topic, 30);
            image_transport::ImageTransport it(nh);
            image_transport::Subscriber subscriber = it.subscribe(in_topic, 1, image_to_compressed_colour);
            ros::spin();
            not_yet_implemented("Colour Compression");
        } else {
            ROS_ERROR_STREAM("Compressing encoding '" << msg_ptr->encoding << "' not currently supported.");
            return 0;
        }
    } else if (info.datatype == "topic_compression/CompressedImage") {
        if (out_topic.empty() || out_topic == "/out")
            out_topic = in_topic + "decompressed";
        ROS_INFO_STREAM("Decompressing colour from '" << in_topic << "' to '" << out_topic << "'");
        publisher = nh.advertise<sensor_msgs::Image>(out_topic, 30);
        ros::Subscriber subscriber = nh.subscribe<topic_compression::CompressedImage>(in_topic, 1,
                                                                                      compressed_colour_to_image);
        ros::spin();
    } else if (info.datatype == "topic_compression/CompressedDepthImage") {
        if (out_topic.empty() || out_topic == "/out")
            out_topic = in_topic + "decompressed";
        ROS_INFO_STREAM("Decompressing depth from '" << in_topic << "' to '" << out_topic << "'");
        publisher = nh.advertise<sensor_msgs::Image>(out_topic, 30);
        ros::Subscriber subscriber = nh.subscribe<topic_compression::CompressedDepthImage>(in_topic, 1,
                                                                                           compressed_depth_to_image);
        ros::spin();
    } else {
        ROS_ERROR_STREAM("No valid compression/decompression for message of type '" << info.datatype << "'");
        return -1;
    }

    return 0;
}