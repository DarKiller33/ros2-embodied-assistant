import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from ultralytics import YOLO
import traceback

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.get_logger().info("Initializing Vision & Perception (YOLOv8) Node...")

        # Load compact YOLO model
        self.model = YOLO('yolov8n.pt')

        # Camera subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.object_pub = self.create_publisher(String, '/detected_objects', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_yolo', 10)

        self.get_logger().info("Subscribed to /camera/image_raw. Vision processing running!")

    def image_callback(self, msg):
        try:
            # 1. Convert raw ROS Image buffer directly to NumPy array (no cv_bridge needed)
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
            if msg.encoding == 'rgb8':
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            # 2. Run YOLOv8 detection
            results = self.model(cv_image, verbose=False)

            detected_classes = set()
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    class_name = self.model.names[cls_id]
                    detected_classes.add(class_name)

            detected_list = list(detected_classes)

            if detected_list:
                msg_str = String()
                msg_str.data = ", ".join(detected_list)
                self.object_pub.publish(msg_str)
                self.get_logger().info(f"Detected: {detected_list}", throttle_duration_sec=2.0)

            # 3. Render bounding boxes onto frame
            annotated_frame = results[0].plot()

            # 4. Construct ROS Image message directly from NumPy array
            yolo_msg = Image()
            yolo_msg.header = msg.header
            yolo_msg.height = annotated_frame.shape[0]
            yolo_msg.width = annotated_frame.shape[1]
            yolo_msg.encoding = 'bgr8'
            yolo_msg.is_bigendian = 0
            yolo_msg.step = annotated_frame.shape[1] * 3
            yolo_msg.data = annotated_frame.tobytes()

            self.image_pub.publish(yolo_msg)

        except Exception:
            self.get_logger().error(f"Error processing frame:\n{traceback.format_exc()}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
