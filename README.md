# Image Converter ROS2 Package

This package implements an image conversion node for ROS2 that subscribes to images from a USB/laptop camera and processes them based on the selected mode (greyscale or color).

## Features

- Subscribes to the image topic published by the `usb_cam` package.
- Converts images to greyscale (Mode 1) or retains color (Mode 2) based on the selected mode.
- Publishes the processed images to a configurable output ROS2 topic.
- Provides a ROS2 service to change the processing mode (greyscale or color) at runtime.
- Launch file starts both the camera node and the image conversion node, with parameters to set input and output topics.
## Demo
 <video controls src="video.mp4" title="Title"></video>
## Installation

1. Ensure you have ROS2 installed on your system.
2. Install the `usb_cam` package to publish images from your USB or laptop camera:
   ```
   sudo apt-get install ros-<ros2-distro>-usb-cam
   ```
3. Clone this repository into your ROS2 workspace:
   ```
   cd <your-ros2-workspace>/src
   git clone <repository-url>
   ```
4. Build the package:
   ```
   cd <your-ros2-workspace>
   colcon build
   ```
5. Source your workspace:
   ```
   source install/setup.bash
   ```

## Usage

1. Launch the nodes using the provided launch file:
   ```
   ros2 launch image_converter_pkg camera_converter.launch.py
   ```
   You can override the input and output topics:
   ```
   ros2 launch image_converter_pkg camera_converter.launch.py input_topic:=/usb_cam/image_raw output_topic:=/image_converter/output_image
   ```
2. Use the ROS2 service to change the mode of the image conversion:
   ```
   ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"  # For greyscale
   ros2 service call /set_mode std_srvs/srv/SetBool "{data: false}" # For color
   ```

## Node Details

- **image_conversion node**: Subscribes to the colored image topic published by `usb_cam`, converts the image to greyscale or keeps it colored based on the mode, and publishes the result to the output topic.
- **Service**: `/set_mode` (type: `std_srvs/srv/SetBool`) — Switches between greyscale (`true`) and color (`false`) modes.

## Package Structure

```
image_converter_pkg
├── image_converter_pkg
│   ├── __init__.py
│   └── image_converter_node.py
├── launch
│   └── camera_converter.launch.py
├── package.xml
├── setup.py
└── README.md
```

## Troubleshooting

- If the `usb_cam` package cannot be installed, ensure your ROS2 distribution is supported and your camera is accessible at `/dev/video0`.
- Check camera permissions: `sudo usermod -a -G video $USER` and then log out/in.

## License

This project is licensed under the MIT License. See the LICENSE file for details.
