import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import os
import time
import importlib.util
import glob

# Add the path to your compiled yolo11_api module - fixed to use absolute path
YOLO_LIB_PATH = "/home/sunrise/Documents/ros2_ws/lib/yolo11_demo/cpp/build"
sys.path.append(YOLO_LIB_PATH)

# Look for the actual .so file dynamically
YOLO_AVAILABLE = False
yolo11_api = None

try:
    # First try direct import
    import yolo11_api
    YOLO_AVAILABLE = True
    print(f"Successfully imported yolo11_api module")
except ImportError as e:
    print(f"Direct import failed: {e}, trying dynamic loading")
    
    # Try to find the .so file
    so_files = glob.glob(f"{YOLO_LIB_PATH}/yolo11_api*.so")
    if so_files:
        so_path = so_files[0]
        print(f"Found module at: {so_path}")
        
        try:
            # Try loading with importlib
            spec = importlib.util.spec_from_file_location("yolo11_api", so_path)
            yolo11_api = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(yolo11_api)
            YOLO_AVAILABLE = True
            print(f"Successfully loaded yolo11_api using importlib")
        except Exception as e:
            print(f"Failed to load module with importlib: {e}")
    else:
        print(f"No yolo11_api*.so files found in {YOLO_LIB_PATH}")
        # List the directory contents for debugging
        print(f"Directory contents: {os.listdir(YOLO_LIB_PATH)}")

# Constants
CONFIDENCE_THRESHOLD = 0.4
SPORTS_BALL_CLASS = 32  # Sports ball class ID in COCO dataset
PERSON_CLASS = 0  # Person class ID in COCO dataset

# Detection modes
MODE_BALL_DETECTION = 0
MODE_PERSON_DETECTION = 1
MODE_NAMES = ["Ball Detection", "Person Detection"]

# Ball selection algorithms
BALL_SELECTION_BOTTOM_EDGE = 0  # Select ball with lowest bottom edge (closest to robot)
BALL_SELECTION_CENTER_PROXIMITY = 1  # Select ball closest to horizontal center
BALL_SELECTION_MODES = ["Bottom Edge Priority", "Center Proximity"]

class FpsCounter:
    """Simple FPS counter for performance monitoring"""
    def __init__(self):
        self.prev_time = time.time()
        self.frames = 0
        self.fps = 0
        self.last_console_print = time.time()
        
    def update(self, print_to_console=False):
        self.frames += 1
        current_time = time.time()
        elapsed = current_time - self.prev_time
        
        if elapsed >= 1.0:
            self.fps = self.frames / elapsed
            self.frames = 0
            self.prev_time = current_time
            
            # If print_to_console is True, print FPS every 2 seconds
            if print_to_console and (current_time - self.last_console_print >= 2.0):
                print(f"Current FPS: {self.fps:.1f}")
                self.last_console_print = current_time
            
        return self.fps

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # Parameters
        self.declare_parameter('show_window', True)
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/yolo/detections')
        self.declare_parameter('control_topic', '/cmd_vel')
        
        self.show_window = self.get_parameter('show_window').get_parameter_value().bool_value
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.control_topic = self.get_parameter('control_topic').get_parameter_value().string_value
        
        # Check if YOLO is available
        if not YOLO_AVAILABLE:
            self.get_logger().error("yolo11_api module not available. Please build the bindings first.")
            return
        
        # Initialize YOLO model
        self.get_logger().info("Initializing YOLO model...")
        if not yolo11_api.initialize_model():
            self.get_logger().error("Failed to initialize YOLO model")
            return
        
        self.get_logger().info("YOLO model initialized successfully")
        
        # Create CV bridge
        self.bridge = CvBridge()
        self.fps_counter = FpsCounter()
        
        # Detection settings
        self.detection_mode = MODE_BALL_DETECTION
        self.ball_selection_mode = BALL_SELECTION_BOTTOM_EDGE
        self.detected_objects = []
        self.best_target = None
        self.detection_confidence = 0
        self.target_class_id = SPORTS_BALL_CLASS
        
        # Create display window if enabled
        if self.show_window:
            self.window_name = "YOLO Detector"
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1280, 720)
            self.get_logger().info("Display window created. Press 'q' to quit, 'm' to switch mode, 'b' to switch ball selection")
        
        # Subscribers
        self.image_subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_publisher = self.create_publisher(
            Image,
            self.output_topic,
            10
        )
        
        self.control_publisher = self.create_publisher(
            Twist,
            self.control_topic,
            10
        )
        
        # Initialize PID controller for object tracking
        if YOLO_AVAILABLE:
            self.pid_controller = yolo11_api.PIDController(0.01, 0.0, 0.005)
        
        self.get_logger().info(f"YOLO Detector Node started - Subscribing to {self.input_topic}")
        
        # Create a timer to check for image reception
        self.last_image_time = None
        self.no_image_warning_timer = self.create_timer(5.0, self.check_for_images)
    
    def image_callback(self, msg):
        if not YOLO_AVAILABLE:
            return
        
        # Update last image time to track when we received data
        self.last_image_time = time.time()
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO inference
            result = yolo11_api.inference(cv_image)
            
            # Update FPS counter
            fps = self.fps_counter.update()
            
            # Process detections
            self.process_detections_results(result, cv_image.shape)
            
            # Draw UI elements and visualize detections
            annotated_image = self.draw_ui(cv_image)
            
            # Publish annotated image
            detection_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            detection_msg.header = msg.header
            self.detection_publisher.publish(detection_msg)
            
            # Display image if enabled
            if self.show_window:
                cv2.imshow(self.window_name, annotated_image)
                key = cv2.waitKey(1) & 0xFF
                self.handle_key_press(key)
            
            # Generate control commands
            self.generate_control_commands(cv_image.shape)
            
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {str(e)}")
    
    def process_detections_results(self, result, image_shape):
        """Process detection results to find target objects"""
        height, width = image_shape[:2]
        
        # Reset detection results
        self.detected_objects = []
        self.best_target = None
        closest_to_center_distance = float('inf')
        lowest_bottom_edge = 0
        
        # Process detection results
        if result and len(result.class_ids) > 0:
            for i, (cls_id, bbox, score) in enumerate(zip(
                    result.class_ids, result.bboxes, result.scores)):
                
                # Skip low confidence detections
                if score < CONFIDENCE_THRESHOLD:
                    continue
                
                x, y, w, h = bbox
                
                # Store all detections
                self.detected_objects.append({
                    'class_id': cls_id,
                    'x': x,
                    'y': y,
                    'width': w,
                    'height': h,
                    'confidence': score
                })
                
                # Only process target objects for the current mode
                if ((self.detection_mode == MODE_BALL_DETECTION and cls_id == SPORTS_BALL_CLASS) or
                    (self.detection_mode == MODE_PERSON_DETECTION and cls_id == PERSON_CLASS)):
                    
                    # Ball selection algorithms
                    if self.ball_selection_mode == BALL_SELECTION_BOTTOM_EDGE:
                        # Get the ball closest to the bottom of the frame
                        ball_bottom_y = y + h
                        
                        if self.best_target is None or ball_bottom_y > lowest_bottom_edge:
                            lowest_bottom_edge = ball_bottom_y
                            self.best_target = (cls_id, x, y, w, h, score)
                            
                    elif self.ball_selection_mode == BALL_SELECTION_CENTER_PROXIMITY:
                        # Get the ball closest to the horizontal center
                        ball_center_x = x + w / 2
                        distance_to_center = abs(ball_center_x - width / 2)
                        
                        if self.best_target is None or distance_to_center < closest_to_center_distance:
                            closest_to_center_distance = distance_to_center
                            self.best_target = (cls_id, x, y, w, h, score)
        
        # Update detection confidence if we have a target
        if self.best_target:
            self.detection_confidence = self.best_target[5]  # confidence is at index 5
        else:
            self.detection_confidence = 0
    
    def handle_key_press(self, key):
        """Handle key presses for interactive control"""
        if key == ord('q'):
            self.get_logger().info("Quitting on user request")
            rclpy.shutdown()
        elif key == ord('m'):
            self.switch_detection_mode()
        elif key == ord('b'):
            self.switch_ball_selection_mode()
        elif key == ord('p'):
            self.target_class_id = PERSON_CLASS
            self.detection_mode = MODE_PERSON_DETECTION
            self.get_logger().info("Switched to Person detection mode")
        elif key == ord('s'):
            self.target_class_id = SPORTS_BALL_CLASS
            self.detection_mode = MODE_BALL_DETECTION
            self.get_logger().info("Switched to Ball detection mode")
    
    def switch_detection_mode(self):
        """Switch between ball and person detection"""
        # Switch detection mode
        self.detection_mode = (self.detection_mode + 1) % len(MODE_NAMES)
        
        # Update target class
        self.target_class_id = SPORTS_BALL_CLASS if self.detection_mode == MODE_BALL_DETECTION else PERSON_CLASS
        
        # Reset detected objects and best target
        self.detected_objects = []
        self.best_target = None
        
        self.get_logger().info(f"Switched to {MODE_NAMES[self.detection_mode]} mode")
    
    def switch_ball_selection_mode(self):
        """Switch between ball selection algorithms"""
        self.ball_selection_mode = (self.ball_selection_mode + 1) % len(BALL_SELECTION_MODES)
        self.get_logger().info(f"Switched to {BALL_SELECTION_MODES[self.ball_selection_mode]} algorithm")
    
    def draw_ui(self, image):
        """Draw UI elements and visualize detections
        
        Args:
            image: Input BGR image
            
        Returns:
            image: Frame with UI elements
        """
        annotated_image = image.copy()
        height, width = annotated_image.shape[:2]
        
        # Draw semi-transparent black background for info panel
        overlay = annotated_image.copy()
        cv2.rectangle(overlay, (10, 10), (320, 100), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, annotated_image, 0.4, 0, annotated_image)
        
        # Display current mode with mode-appropriate color
        mode_color = (0, 0, 255) if self.detection_mode == MODE_BALL_DETECTION else (0, 255, 0)
        cv2.putText(annotated_image, f"Mode: {MODE_NAMES[self.detection_mode]}", (20, 35), 
                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)
        
        # Display ball selection mode if in ball detection mode
        if self.detection_mode == MODE_BALL_DETECTION:
            cv2.putText(annotated_image, f"Selection: {BALL_SELECTION_MODES[self.ball_selection_mode]}", (20, 65), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Display FPS
        fps = self.fps_counter.fps
        cv2.putText(annotated_image, f"FPS: {fps:.1f}", (20, 95), 
                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                  
        # Display detection status if we have a best target
        if self.best_target:
            cls_id = self.best_target[0]
            if self.detection_mode == MODE_BALL_DETECTION and cls_id == SPORTS_BALL_CLASS:
                status = f"Ball detected: {int(self.detection_confidence * 100)}%"
                status_color = (0, 0, 255)  # Red for balls
            elif self.detection_mode == MODE_PERSON_DETECTION and cls_id == PERSON_CLASS:
                status = f"Person detected: {int(self.detection_confidence * 100)}%"
                status_color = (0, 255, 0)  # Green for people
            else:
                status = "Target detected"
                status_color = (255, 255, 255)
                
            cv2.putText(annotated_image, status, (20, 125), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        else:
            # No detection message with appropriate color
            if self.detection_mode == MODE_BALL_DETECTION:
                cv2.putText(annotated_image, "No ball detected", (20, 125), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(annotated_image, "No person detected", (20, 125), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw instructions at the bottom
        cv2.putText(annotated_image, "Q: Quit | M: Switch mode | B: Switch selection | P: Person mode | S: Sports ball mode",
                   (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Get the target class ID for the current detection mode
        target_class_id = self.target_class_id
        
        # Draw all detected objects
        for obj in self.detected_objects:
            cls_id = obj['class_id']
            x, y, w, h = obj['x'], obj['y'], obj['width'], obj['height']
            
            # Choose color and label based on class
            if cls_id == SPORTS_BALL_CLASS:
                color = (0, 0, 255)  # Red for balls
                label = f"Ball: {int(obj['confidence'] * 100)}%"
            elif cls_id == PERSON_CLASS:
                color = (0, 255, 0)  # Green for people
                label = f"Person: {int(obj['confidence'] * 100)}%"
            else:
                color = (255, 255, 255)  # White for other objects
                label = f"Class {cls_id}: {int(obj['confidence'] * 100)}%"
            
            # Draw bounding box
            cv2.rectangle(annotated_image, (int(x), int(y)), (int(x + w), int(y + h)), color, 2)
            
            # Add label
            cv2.putText(annotated_image, label, (int(x), int(y - 10)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw additional visualization for best target
        if self.best_target:
            cls_id, x, y, w, h, conf = self.best_target
            
            if self.detection_mode == MODE_BALL_DETECTION:
                # Ball tracking visualization
                ball_center_x = x + w / 2
                ball_center_y = y + h / 2
                
                # Draw center point of the ball
                cv2.circle(annotated_image, (int(ball_center_x), int(ball_center_y)), 5, (0, 255, 255), -1)
                
                # Calculate target position (bottom center of frame)
                target_x = width / 2
                target_y = height * 0.9  # 90% down the frame
                
                # Draw target position
                cv2.circle(annotated_image, (int(target_x), int(target_y)), 10, (255, 255, 0), 2)
                cv2.line(annotated_image, (int(target_x - 15), int(target_y)), 
                        (int(target_x + 15), int(target_y)), (255, 255, 0), 2)
                cv2.line(annotated_image, (int(target_x), int(target_y - 15)), 
                        (int(target_x), int(target_y + 15)), (255, 255, 0), 2)
                
                # Draw line from ball to target
                cv2.line(annotated_image, (int(ball_center_x), int(ball_center_y)), 
                        (int(target_x), int(target_y)), (0, 255, 255), 2)
                
            elif self.detection_mode == MODE_PERSON_DETECTION:
                # Person tracking visualization
                person_center_x = x + w / 2
                person_center_y = y + h / 2
                
                # Draw center point of the person
                cv2.circle(annotated_image, (int(person_center_x), int(person_center_y)), 5, (255, 150, 0), -1)
                
                # Calculate center of frame
                frame_center_x = width / 2
                
                # Draw a vertical line at frame center for reference
                cv2.line(annotated_image, (int(frame_center_x), 0), (int(frame_center_x), height), 
                        (0, 150, 255), 1, cv2.LINE_AA)
                
                # Draw line from person to center line
                cv2.line(annotated_image, (int(person_center_x), int(person_center_y)), 
                        (int(frame_center_x), int(person_center_y)), (0, 255, 255), 2)
        
        return annotated_image
    
    def generate_control_commands(self, image_shape):
        """Generate control commands based on detections of target class"""
        twist = Twist()
        
        # If we don't have a best target, stop
        if not self.best_target:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.control_publisher.publish(twist)
            return
        
        # Unpack best target
        cls_id, x, y, w, h, conf = self.best_target
        
        # Only control if confidence is high enough and class ID matches target
        if conf > CONFIDENCE_THRESHOLD and cls_id == self.target_class_id:
            # Calculate object center
            object_center_x = x + w / 2
            image_center_x = image_shape[1] / 2  # Image width / 2
            
            # Calculate error for PID controller
            error = object_center_x - image_center_x
            
            # Use PID controller for smooth turning
            angular_velocity = self.pid_controller.calculate(error)
            
            # Set control commands
            twist.linear.x = 0.2  # Move forward
            twist.angular.z = -angular_velocity  # Turn to center object
            
            target_name = "ball" if cls_id == SPORTS_BALL_CLASS else "person"
            self.get_logger().info(f"Tracking {target_name}: error={error:.2f}, angular_vel={angular_velocity:.3f}")
        else:
            # Stop if no high-confidence detection
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.control_publisher.publish(twist)
    
    def check_for_images(self):
        """Check if we're receiving images, warn if not"""
        # Add instance variables to track image reception
        if not hasattr(self, 'last_image_time'):
            self.last_image_time = None
            self.no_image_warning_timer = self.create_timer(5.0, self.check_for_images)
            return
        
        current_time = time.time()
        
        if self.last_image_time is None or (current_time - self.last_image_time) > 5.0:
            self.get_logger().warn("No images received for 5 seconds. Make sure an image topic is being published.")
            self.get_logger().warn(f"Subscribed to: {self.input_topic}")
            self.get_logger().warn("You can check available topics with: ros2 topic list | grep image")
            
            # Display an information frame if we're showing a window
            if self.show_window:
                # Create an information frame
                info_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(info_frame, "Waiting for images...", (50, 50), 
                          cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(info_frame, f"Subscribed to: {self.input_topic}", (50, 100), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(info_frame, "No images received", (50, 150), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                cv2.putText(info_frame, "Press 'q' to quit", (50, 200), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                cv2.imshow(self.window_name, info_frame)
                key = cv2.waitKey(1) & 0xFF
                self.handle_key_press(key)
        
        # Update the time of the last received image
        self.last_image_time = current_time
    
    def __del__(self):
        """Cleanup when node is destroyed"""
        if hasattr(self, 'show_window') and self.show_window:
            cv2.destroyAllWindows()
        
        if YOLO_AVAILABLE:
            yolo11_api.cleanup_model()

def main(args=None):
    rclpy.init(args=args)
    
    node = YoloDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'show_window') and node.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()