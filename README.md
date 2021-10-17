# Topic Compression

![CI](https://github.com/RaymondKirk/topic_compression/workflows/Topic%20Compression/badge.svg?branch=main)

Compress common ROS topic messages to save space or bandwidth.


## Installation

```bash
# From source
cd catkin_ws/src
git clone https://github.com/RaymondKirk/topic_compression 
catkin build topic_compression
```

## Usage 

Pass the topic to compress/decompress (in) and a output topic to publish on. 

```bash
# For example to compress
rosrun topic_compression run in:="/camera/depth/image_raw" out:="/camera/depth/compressed"

# To decompress it's the same syntax
rosrun topic_compression run in:="/camera/depth/compressed" out:="/camera/depth/image_raw_1"
```

### Python Wrapper

Decompress your image data in Python.

```python
from topic_compression.msg import CompressedDepthImage, CompressedImage
from topic_compression_lib import compress_image, decompress_image, compress_depth, decompress_depth

# Your compressed data
compressed_image = CompressedImage(...)
compressed_depth = CompressedDepthImage(...)

# Decompress in python (library is built with catkin build)
decompressed_image = decompress_image(compressed_image)
decompressed_depth_image = decompress_depth(compressed_depth)
```