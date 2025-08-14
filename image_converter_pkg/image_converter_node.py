import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2

class ImageConverterNode(Node):
    def __init__(self):
        super().__init__('image_converter_node')
        self.bridge = CvBridge()
        self.mode = True  # False: Color, True: Grayscale

        # Declare parameters for input and output topics
        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('output_topic', '/image_converter/output_image')
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Create subscription and publisher
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        self.image_pub = self.create_publisher(
            Image,
            output_topic,
            10
        )

        # Create service for mode switching
        self.service = self.create_service(
            SetBool,
            '/set_mode',
            self.set_mode_callback
        )

        # Name the OpenCV window
        cv2.namedWindow("Image Converter Output", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        try:
            # Try direct BGR8 conversion
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        if self.mode:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='mono8')
            display_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        else:
            output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            display_image = cv_image.copy()

        output_msg.header = msg.header
        self.image_pub.publish(output_msg)

        # Draw mode text
        mode_text = "Mode: Grayscale" if self.mode else "Mode: Color"
        color = (0, 255, 0) if not self.mode else (200, 200, 200)
        cv2.putText(display_image, mode_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)

        cv2.imshow("Image Converter Output", display_image)
        cv2.waitKey(1)


    def set_mode_callback(self, request, response):
        self.mode = request.data
        self.get_logger().info(f'Set mode to {"Grayscale" if self.mode else "Color"}')
        response.success = True
        response.message = f'Mode set to {"Grayscale" if self.mode else "Color"}'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ImageConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
