#include "bidirectional.cpp"

static inline bool is_ros_msg_type(pybind11::handle src, const std::string &msg_type_name) {
    return pybind11::hasattr(src, "_type") && src.attr("_type").cast<std::string>() == msg_type_name;
}

namespace pybind11 {
    namespace detail {
        // Two way casting inspired by https://github.com/dimatura/pyrosmsg
        template<>
        struct type_caster<ros::Time> {
        public:
            PYBIND11_TYPE_CASTER(ros::Time, _("ros::Time"));
            // python -> cpp
            bool load(handle src, bool) {
                PyObject *obj(src.ptr());
                if (!PyObject_HasAttrString(obj, "secs") || !PyObject_HasAttrString(obj, "nsecs")) {
                    return false;
                }

                value.sec = (src.attr("secs")).cast<uint32_t>();
                value.nsec = (src.attr("nsecs")).cast<uint32_t>();
                return true;
            }

            // cpp -> python
            static handle cast(ros::Time src, return_value_policy policy, handle parent) {
                object rospy = module::import("rospy");
                object TimeType = rospy.attr("Time");
                object pyts = TimeType();
                pyts.attr("secs") = pybind11::cast(src.sec);
                pyts.attr("nsecs") = pybind11::cast(src.nsec);
                pyts.inc_ref();
                return pyts;
            }
        };

        template<>
        struct type_caster<std_msgs::Header> {
        public:
            PYBIND11_TYPE_CASTER(std_msgs::Header, _("std_msgs::Header"));
            // python -> cpp
            bool load(handle src, bool) {
                if (!is_ros_msg_type(src, "std_msgs/Header")) {
                    return false;
                }
                value.seq = src.attr("seq").cast<uint32_t>();
                value.stamp = src.attr("stamp").cast<ros::Time>();
                value.frame_id = src.attr("frame_id").cast<std::string>();
                return true;
            }

            // cpp -> python
            static handle cast(std_msgs::Header header,
                               return_value_policy policy,
                               handle parent) {
                object mod = module::import("std_msgs.msg._Header");
                object MsgType = mod.attr("Header");
                object msg = MsgType();
                msg.attr("seq") = pybind11::cast(header.seq);
                msg.attr("stamp") = pybind11::cast(header.stamp);
                msg.attr("frame_id") =
                        pybind11::bytes(reinterpret_cast<const char *>(&header.frame_id[0]),
                                        header.frame_id.size());
                msg.inc_ref();
                return msg;
            }
        };

        template<>
        struct type_caster<sensor_msgs::Image> {
        public:
            PYBIND11_TYPE_CASTER(sensor_msgs::Image, _("sensor_msgs::Image"));

            bool load(handle src, bool) {
                if (!is_ros_msg_type(src, "sensor_msgs/Image")) {
                    return false;
                }
                value.header = src.attr("header").cast<std_msgs::Header>();
                value.height = src.attr("height").cast<uint32_t>();
                value.width = src.attr("width").cast<uint32_t>();
                value.encoding = src.attr("encoding").cast<std::string>();
                value.is_bigendian = src.attr("is_bigendian").cast<int>();
                value.step = src.attr("step").cast<uint32_t>();
                std::string data_str = src.attr("data").cast<std::string>();
                value.data.insert(value.data.end(),
                                  data_str.c_str(),
                                  data_str.c_str() + data_str.length());
                return true;
            }

            static handle cast(sensor_msgs::Image cpp_msg,
                               return_value_policy policy,
                               handle parent) {
                object mod = module::import("sensor_msgs.msg._Image");
                object MsgType = mod.attr("Image");
                object msg = MsgType();
                msg.attr("header") = pybind11::cast(cpp_msg.header);
                msg.attr("height") = pybind11::cast(cpp_msg.height);
                msg.attr("width") = pybind11::cast(cpp_msg.width);
                msg.attr("encoding") = pybind11::bytes(cpp_msg.encoding);
                msg.attr("is_bigendian") = pybind11::cast(cpp_msg.is_bigendian);
                msg.attr("step") = pybind11::cast(cpp_msg.step);
                msg.attr("data") = pybind11::bytes(
                        reinterpret_cast<const char *>(&cpp_msg.data[0]), cpp_msg.data.size());
                msg.inc_ref();
                return msg;
            }
        };

        template<>
        struct type_caster<sensor_msgs::CompressedImage> {
        public:
            PYBIND11_TYPE_CASTER(sensor_msgs::CompressedImage, _("sensor_msgs::CompressedImage"));

            bool load(handle src, bool) {
                if (!is_ros_msg_type(src, "sensor_msgs/CompressedImage")) {
                    return false;
                }
                value.header = src.attr("header").cast<std_msgs::Header>();
                value.format = src.attr("format").cast<std::string>();
                std::string data_str = src.attr("data").cast<std::string>();
                value.data.insert(value.data.end(),
                                  data_str.c_str(),
                                  data_str.c_str() + data_str.length());
                return true;
            }

            static handle cast(sensor_msgs::CompressedImage cpp_msg,
                               return_value_policy policy,
                               handle parent) {
                object mod = module::import("sensor_msgs.msg._CompressedImage");
                object MsgType = mod.attr("CompressedImage");
                object msg = MsgType();
                msg.attr("header") = pybind11::cast(cpp_msg.header);
                msg.attr("format") = pybind11::bytes(cpp_msg.format);
                msg.attr("data") = pybind11::bytes(reinterpret_cast<const char *>(&cpp_msg.data[0]),
                                                   cpp_msg.data.size());
                msg.inc_ref();
                return msg;
            }
        };

        template<>
        struct type_caster<sensor_msgs::CameraInfo> {
        public:
            PYBIND11_TYPE_CASTER(sensor_msgs::CameraInfo, _("sensor_msgs::CameraInfo"));

            bool load(handle src, bool) {
                if (!is_ros_msg_type(src, "sensor_msgs/CameraInfo")) {
                    return false;
                }

                value.height = src.attr("height").cast<uint32_t>();
                value.width = src.attr("width").cast<uint32_t>();
                value.distortion_model = src.attr("distortion_model").cast<std::string>();

                {
                    for (auto item : src.attr("D")) {
                        value.D.push_back(item.cast<double>());
                    }
                }
                {
                    int i = 0;
                    for (auto item : src.attr("K")) {
                        value.K[i] = item.cast<double>();
                        ++i;
                    }
                }
                {
                    int i = 0;
                    for (auto item : src.attr("R")) {
                        value.R[i] = item.cast<double>();
                        ++i;
                    }
                }
                {
                    int i = 0;
                    for (auto item : src.attr("P")) {
                        value.P[i] = item.cast<double>();
                        ++i;
                    }
                }

                value.header = src.attr("header").cast<std_msgs::Header>();

                return true;
            }

            static handle cast(sensor_msgs::CameraInfo cpp_msg,
                               return_value_policy policy,
                               handle parent) {
                object mod = module::import("sensor_msgs.msg._CameraInfo");
                object MsgType = mod.attr("CameraInfo");
                object msg = MsgType();

                // TODO untested

                msg.attr("height") = cpp_msg.height;
                msg.attr("width") = cpp_msg.width;

                msg.attr("distortion_model") = cpp_msg.distortion_model;

                for (size_t i = 0; i < cpp_msg.D.size(); ++i) {
                    pybind11::list D = msg.attr("D");
                    D[i] = cpp_msg.K[i];
                }

                for (size_t i = 0; i < cpp_msg.K.size(); ++i) {
                    pybind11::list K = msg.attr("K");
                    K[i] = cpp_msg.K[i];
                }

                for (size_t i = 0; i < cpp_msg.R.size(); ++i) {
                    pybind11::list R = msg.attr("R");
                    R[i] = cpp_msg.K[i];
                }

                for (size_t i = 0; i < cpp_msg.P.size(); ++i) {
                    pybind11::list P = msg.attr("P");
                    P[i] = cpp_msg.P[i];
                }

                msg.attr("header") = pybind11::cast(cpp_msg.header);

                msg.inc_ref();
                return msg;
            }
        };

        template<>
        struct type_caster<topic_compression::CompressionMeta> {
        public:
            PYBIND11_TYPE_CASTER(topic_compression::CompressionMeta, _("topic_compression::CompressionMeta"));

            // python -> cpp
            bool load(handle src, bool) {
                if (!is_ros_msg_type(src, "topic_compression/CompressionMeta")) {
                    return false;
                }
                value.header = src.attr("header").cast<std_msgs::Header>();
                value.height = src.attr("height").cast<uint32_t>();
                value.width = src.attr("width").cast<uint32_t>();
                value.encoding = src.attr("encoding").cast<std::string>();
                value.is_bigendian = src.attr("is_bigendian").cast<int>();
                value.step = src.attr("step").cast<uint32_t>();
                value.algorithm = src.attr("algorithm").cast<std::string>();
                return true;
            }

            // cpp -> python
            static handle cast(topic_compression::CompressionMeta cpp_msg,
                               return_value_policy policy,
                               handle parent) {
                object mod = module::import("topic_compression.msg._CompressionMeta");
                object MsgType = mod.attr("CompressionMeta");
                object msg = MsgType();
                msg.attr("header") = pybind11::cast(cpp_msg.header);
                msg.attr("height") = pybind11::cast(cpp_msg.height);
                msg.attr("width") = pybind11::cast(cpp_msg.width);
                msg.attr("encoding") = pybind11::bytes(cpp_msg.encoding);
                msg.attr("is_bigendian") = pybind11::cast(cpp_msg.is_bigendian);
                msg.attr("step") = pybind11::cast(cpp_msg.step);
                msg.attr("algorithm") = pybind11::bytes(cpp_msg.algorithm);
                msg.inc_ref();
                return msg;
            }
        };

        template<>
        struct type_caster<topic_compression::CompressedImage> {
        public:
            PYBIND11_TYPE_CASTER(topic_compression::CompressedImage, _("topic_compression::CompressedImage"));

            bool load(handle src, bool) {
                if (!is_ros_msg_type(src, "topic_compression/CompressedImage")) {
                    return false;
                }
                value.header = src.attr("header").cast<std_msgs::Header>();
                value.data = src.attr("data").cast<sensor_msgs::CompressedImage>();
                value.meta = src.attr("meta").cast<topic_compression::CompressionMeta>();
                return true;
            }

            static handle cast(topic_compression::CompressedImage cpp_msg,
                               return_value_policy policy,
                               handle parent) {
                object mod = module::import("topic_compression.msg._CompressedImage");
                object MsgType = mod.attr("CompressedImage");
                object msg = MsgType();
                msg.attr("header") = pybind11::cast(cpp_msg.header);
                msg.attr("data") = pybind11::cast(cpp_msg.data);;
                msg.attr("meta") = pybind11::cast(cpp_msg.meta);
                msg.inc_ref();
                return msg;
            }
        };

        template<>
        struct type_caster<topic_compression::CompressedDepthImage> {
        public:
            PYBIND11_TYPE_CASTER(topic_compression::CompressedDepthImage, _("topic_compression::CompressedDepthImage"));

            bool load(handle src, bool) {
                if (!is_ros_msg_type(src, "topic_compression/CompressedDepthImage")) {
                    return false;
                }
                value.header = src.attr("header").cast<std_msgs::Header>();
                std::string data_str = src.attr("data").cast<std::string>();
                value.data.insert(value.data.end(),
                                  data_str.c_str(),
                                  data_str.c_str() + data_str.length());
                value.meta = src.attr("meta").cast<topic_compression::CompressionMeta>();
                return true;
            }

            static handle cast(topic_compression::CompressedDepthImage cpp_msg,
                               return_value_policy policy,
                               handle parent) {
                object mod = module::import("topic_compression.msg._CompressedDepthImage");
                object MsgType = mod.attr("CompressedDepthImage");
                object msg = MsgType();
                msg.attr("header") = pybind11::cast(cpp_msg.header);
                msg.attr("meta") = pybind11::cast(cpp_msg.meta);
                msg.attr("data") = pybind11::bytes(reinterpret_cast<const char *>(&cpp_msg.data[0]),
                                                   cpp_msg.data.size());
                msg.inc_ref();
                return msg;
            }
        };
    }
}

namespace topic_compression_lib {
    topic_compression::CompressedImage compress_image(const sensor_msgs::Image& cpp_msg) {
        throw std::logic_error("Image compression not yet implemented.");
        sensor_msgs::Image::ConstPtr msg_ptr(new sensor_msgs::Image(cpp_msg));
        return image_to_compressed_colour(msg_ptr);
    }

    topic_compression::CompressedDepthImage compress_depth(const sensor_msgs::Image& cpp_msg) {
        throw std::logic_error("Depth compression not yet implemented.");
        sensor_msgs::Image::ConstPtr msg_ptr(new sensor_msgs::Image(cpp_msg));
        return image_to_compressed_depth(msg_ptr);
    }

    sensor_msgs::Image decompress_image(const topic_compression::CompressedImage& cpp_msg) {
        topic_compression::CompressedImage::ConstPtr msg_ptr(new topic_compression::CompressedImage(cpp_msg));
        return compressed_colour_to_image(msg_ptr);
    }

    sensor_msgs::Image decompress_depth(const topic_compression::CompressedDepthImage& cpp_msg) {
        topic_compression::CompressedDepthImage::ConstPtr cmp_ptr(new topic_compression::CompressedDepthImage(cpp_msg));
        return compressed_depth_to_image(cmp_ptr);
    }
}

PYBIND11_MODULE(topic_compression_lib, m) {
    m.doc() = "Topic Compression Python Binding";
    m.def("compress_image", &topic_compression_lib::compress_image, "Compress ROS image message");
    m.def("compress_depth", &topic_compression_lib::compress_depth, "Compress ROS depth image message");
    m.def("decompress_image", &topic_compression_lib::decompress_image, "Compressed image to ROS image message");
    m.def("decompress_depth", &topic_compression_lib::decompress_depth, "Compressed depth to ROS image message");
}