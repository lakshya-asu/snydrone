import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge

# Use Ultralytics YOLOv8 for real-time tracking
try:
    from ultralytics import YOLO
    HAS_YOLO = True
except ImportError:
    HAS_YOLO = False

import json

class TrackerNode(Node):
    def __init__(self):
        super().__init__('snydrone_tracker_node')

        self.get_logger().info("Initializing Snydrone Vision Tracker...")
        
        # Parameters
        self.declare_parameter('model', 'yolov8s-world.pt')
        self.declare_parameter('target_class', 'cardboard box') # e.g. 'person', 'car', 'truck'
        self.declare_parameter('confidence', 0.5)

        model_name = self.get_parameter('model').value
        self.target_class_name = self.get_parameter('target_class').value
        self.conf_threshold = self.get_parameter('confidence').value

        self.bridge = CvBridge()

        # Load YOLO model
        if HAS_YOLO:
            self.get_logger().info(f"Loading YOLO Model: {model_name}")
            self.model = YOLO(model_name)
            
            # For open-vocabulary models like YOLO-World, set the custom classes
            if 'world' in model_name.lower():
                self.model.set_classes([self.target_class_name])
                self.target_class_id = 0  # The only class we set
                self.get_logger().info(f"YOLO-World Open Vocabulary classes set to: {[self.target_class_name]}")
            else:
                self.class_names = self.model.names
                # Find the ID for the target class
                self.target_class_id = None
                for idx, name in self.class_names.items():
                    if name.lower() == self.target_class_name.lower():
                        self.target_class_id = idx
                        break
                
                if self.target_class_id is None:
                    self.get_logger().error(f"Target class '{self.target_class_name}' not found in model classes!")
        else:
            self.get_logger().error("Ultralytics YOLO not installed. Tracking disabled.")
            self.model = None

        # Subscribers & Publishers
        # Pegasus monocular camera default topic is usually /drone1/camera/color/image_raw
        self.declare_parameter('camera_topic', '/drone1/camera/color/image_raw')
        cam_topic = self.get_parameter('camera_topic').value
        
        # Subscribe to camera
        self.sub_cam = self.create_subscription(
            Image,
            cam_topic,
            self.camera_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        # Publish tracking info (e.g., bounding box center [x, y] in normalized image coordinates -1 to 1)
        # Using a JSON string for easy extensibility or Point for direct x,y
        self.pub_track = self.create_publisher(Point, '/snydrone/vision/track_center', 10)
        
        # Publish annotated image stream for debugging/visualization
        self.pub_vis = self.create_publisher(Image, '/snydrone/vision/annotated_image', 1)

        self.get_logger().info(f"Tracker online. Listening to {cam_topic}, looking for '{self.target_class_name}'")

    def camera_callback(self, msg: Image):
        if not HAS_YOLO or self.model is None or self.target_class_id is None:
            return

        try:
            # Convert ROS Image to OpenCV image (BGR)
            # Pegasus simulator rgb images are sometimes RGBA
            if msg.encoding == "rgba8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        h, w = cv_image.shape[:2]

        # Run YOLO inference
        results = self.model(cv_image, verbose=False, conf=self.conf_threshold)

        best_box = None
        max_area = 0

        # Parse results
        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                if cls_id == self.target_class_id:
                    # Get bounding box coordinates [x1, y1, x2, y2]
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    area = (x2 - x1) * (y2 - y1)
                    
                    # Track the largest target (closest)
                    if area > max_area:
                        max_area = area
                        best_box = (x1, y1, x2, y2)
        
        if best_box is not None:
            x1, y1, x2, y2 = best_box
            
            # Calculate pixel center
            cx_pixel = (x1 + x2) / 2.0
            cy_pixel = (y1 + y2) / 2.0

            # Normalize center to [-1.0, 1.0] range where 0,0 is center of image
            # Right is +x, Down is +y
            cx_norm = (cx_pixel - w / 2.0) / (w / 2.0)
            cy_norm = (cy_pixel - h / 2.0) / (h / 2.0)

            # Publish
            track_msg = Point()
            track_msg.x = float(cx_norm)
            track_msg.y = float(cy_norm)
            track_msg.z = float(max_area / (w * h)) # z holds normalized area (size of object)
            self.pub_track.publish(track_msg)

            # Visualize
            cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(cv_image, (int(cx_pixel), int(cy_pixel)), 5, (0, 0, 255), -1)
            cv2.putText(cv_image, f"{self.target_class_name} ({cx_norm:.2f}, {cy_norm:.2f})", 
                        (int(x1), int(max(0, y1-10))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.pub_vis.publish(annotated_msg)
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
