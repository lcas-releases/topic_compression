# This message is for storing compressed depth image
#  Raymond Kirk (Tunstill) Copyright (c) 2021
#  Email: ray.tunstill@gmail.com

# New header (note. use meta.header to get the original timestamp
std_msgs/Header header

# Compressed data
sensor_msgs/CompressedImage data

# Original meta from the original image
CompressionMeta meta

