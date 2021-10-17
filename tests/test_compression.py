from topic_compression.msg import CompressedDepthImage, CompressedImage
from sensor_msgs.msg import Image
from topic_compression_lib import compress_image, decompress_image, compress_depth, decompress_depth

# Your compressed data
compressed_image = CompressedImage()
compressed_depth = CompressedDepthImage()

# Decompress in python (library is built with catkin build)
#decompressed_image = decompress_image(compressed_image)
#decompressed_depth_image = decompress_depth(compressed_depth)
