import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import glob
import os
import threading

class SimpleFpsCounter:
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

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Camera settings
        self.camera = None
        self.camera_id = 0
        self.frame_width = 1280
        self.frame_height = 712
        self.is720p = True
        
        # View mode settings (0: RGB, 1: Grayscale, 2: Edge detection, 3: Blur)
        self.view_mode = 0
        self.view_mode_names = ["RGB", "Grayscale", "Edge Detection", "Blur"]
        
        # Preview window settings
        self.window_name = "Camera Publisher Preview"
        self.show_preview = True
        self.is_fullscreen = False
        self.print_fps_to_console = False
        
        # FPS counter
        self.fps_counter = SimpleFpsCounter()
        
        # Thread settings
        self.running = True
        self.publish_thread = None
        self.camera_lock = threading.Lock()
        
        # Initialize camera
        self.get_logger().info("Finding available camera...")
        camera_found = self.find_and_open_camera()
        
        if camera_found:
            self.get_logger().info("Camera Publisher Node has been started.")
            self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 Hz
            
            # Setup preview window
            if self.show_preview:
                self.setup_preview_window()
                self.get_logger().info("Preview window initialized")
                self.get_logger().info("Press 'q' to quit, 'r' to toggle resolution, " +
                             "'f' to toggle fullscreen, 'm' to switch view mode, " +
                             "'p' to toggle preview")
        else:
            self.get_logger().error("Failed to open camera")

    def find_available_camera(self):
        """Find an available camera"""
        # Try common camera indices
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.get_logger().info(f"Found working camera at index {i}")
                    cap.release()
                    return i
                cap.release()
        
        # If no camera found with standard indices, try device paths (Linux)
        if os.path.exists("/dev/"):
            video_devices = glob.glob("/dev/video*")
            for device in video_devices:
                try:
                    # Extract number from device path
                    device_num = int(device.replace("/dev/video", ""))
                    cap = cv2.VideoCapture(device_num)
                    if cap.isOpened():
                        ret, frame = cap.read()
                        if ret and frame is not None:
                            self.get_logger().info(f"Found working camera at {device} (index {device_num})")
                            cap.release()
                            return device_num
                    cap.release()
                except Exception as e:
                    self.get_logger().error(f"Error checking {device}: {e}")
        
        self.get_logger().error("No working camera found")
        return None

    def find_and_open_camera(self):
        """Find and open camera"""
        self.camera_id = self.find_available_camera()
        if self.camera_id is None:
            return False
        
        self.camera = cv2.VideoCapture(self.camera_id)
        if not self.camera.isOpened():
            return False
        
        # Set camera properties
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer for reduced latency
        
        # Get actual resolution
        actual_width = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Camera opened with resolution: {actual_width}x{actual_height}")
        
        return True

    def setup_preview_window(self):
        """Setup the preview window"""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        # Try to get screen resolution
        try:
            # Try to get screen resolution using xrandr (Linux)
            import subprocess
            output = subprocess.check_output('xrandr | grep "\\*" | cut -d" " -f4', shell=True).decode('utf-8').strip()
            screen_w, screen_h = map(int, output.split('x'))
        except:
            # Fallback to a common resolution
            screen_w, screen_h = 1920, 1080
            
        # Get actual camera resolution
        if self.camera and self.camera.isOpened():
            actual_width = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
        else:
            actual_width, actual_height = self.frame_width, self.frame_height
            
        # Calculate window size (80% of screen)
        window_w = int(screen_w * 0.8)
        window_h = int(window_w * actual_height / actual_width)
        
        if window_h > screen_h * 0.8:
            window_h = int(screen_h * 0.8)
            window_w = int(window_h * actual_width / actual_height)
            
        cv2.resizeWindow(self.window_name, window_w, window_h)
        
        # Position window in center of screen
        win_x = (screen_w - window_w) // 2
        win_y = (screen_h - window_h) // 2
        cv2.moveWindow(self.window_name, win_x, win_y)

    def toggle_resolution(self):
        """Toggle camera resolution"""
        if not self.camera or not self.camera.isOpened():
            return self.is720p
            
        with self.camera_lock:
            # Set properties that affect switching delay
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            if self.is720p:
                # Switch to 712p
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 712)
                
                # Flush the buffer
                for _ in range(2):
                    self.camera.grab()
                    
                self.get_logger().info("Resolution changed to 1280x712")
            else:
                # Switch to 720p
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                
                # Flush the buffer
                for _ in range(2):
                    self.camera.grab()
                    
                self.get_logger().info("Resolution changed to 1280x720")
            
            # Get actual resolution
            actual_width = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.get_logger().info(f"Actual resolution: {actual_width}x{actual_height}")
            
            self.is720p = not self.is720p
            return self.is720p

    def toggle_fullscreen(self):
        """Toggle fullscreen mode for preview window"""
        if not self.show_preview:
            return False
            
        self.is_fullscreen = not self.is_fullscreen
        
        if self.is_fullscreen:
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            self.get_logger().info("Switched to fullscreen mode")
        else:
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
            self.get_logger().info("Exited fullscreen mode")
            
        return self.is_fullscreen
        
    def toggle_preview(self):
        """Toggle preview window on/off"""
        self.show_preview = not self.show_preview
        
        if self.show_preview:
            self.setup_preview_window()
            self.get_logger().info("Preview window enabled")
            self.print_fps_to_console = False
        else:
            cv2.destroyWindow(self.window_name)
            self.get_logger().info("Preview window disabled, FPS will be printed to console")
            self.print_fps_to_console = True
            
        return self.show_preview
        
    def switch_view_mode(self):
        """Switch between different view modes"""
        self.view_mode = (self.view_mode + 1) % len(self.view_mode_names)
        self.get_logger().info(f"Switched to {self.view_mode_names[self.view_mode]} mode")
        return self.view_mode

    def process_frame(self, frame):
        """Process the frame based on the current view mode"""
        if self.view_mode == 0:  # RGB original
            processed_frame = frame
        elif self.view_mode == 1:  # Grayscale
            processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            processed_frame = cv2.cvtColor(processed_frame, cv2.COLOR_GRAY2BGR)  # Convert back to BGR for display/publishing
        elif self.view_mode == 2:  # Edge detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 100, 200)
            processed_frame = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)  # Convert back to BGR for display/publishing
        elif self.view_mode == 3:  # Blur
            processed_frame = cv2.GaussianBlur(frame, (15, 15), 0)
        else:
            processed_frame = frame
            
        return processed_frame

    def draw_ui(self, frame):
        """Draw UI elements on the frame"""
        height, width = frame.shape[:2]
        
        # Draw semi-transparent black background for info panel
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (320, 100), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        
        # Display current mode
        cv2.putText(frame, f"Mode: {self.view_mode_names[self.view_mode]}", (20, 35), 
                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Display FPS
        fps = self.fps_counter.update()
        cv2.putText(frame, f"FPS: {fps:.1f}", (20, 65), 
                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                  
        # Display camera resolution
        if self.camera and self.camera.isOpened():
            actual_width = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
            resolution_text = f"Resolution: {int(actual_width)}x{int(actual_height)}"
            cv2.putText(frame, resolution_text, (20, 95), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return frame

    def timer_callback(self):
        """Timer callback to capture and publish camera frames"""
        if not self.camera or not self.camera.isOpened():
            self.get_logger().warning("Camera not available")
            return
            
        with self.camera_lock:
            # Capture frame
            ret, frame = self.camera.read()
            
        if not ret or frame is None:
            self.get_logger().warning("Failed to capture image")
            return
            
        # Process the frame based on view mode
        processed_frame = self.process_frame(frame)
        
        # Convert to ROS Image message and publish
        msg = self.bridge.cv2_to_imgmsg(processed_frame, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().debug('Publishing image')
        
        # Handle preview window
        if self.show_preview:
            # Add UI elements to displayed frame
            display_frame = self.draw_ui(processed_frame.copy())
            
            # Display the processed frame
            cv2.imshow(self.window_name, display_frame)
            
            # Check for key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                # Clean shutdown
                self.get_logger().info("Shutdown requested by user")
                self.destroy_node()
                rclpy.shutdown()
            elif key == ord('r'):
                self.toggle_resolution()
            elif key == ord('f'):
                self.toggle_fullscreen()
            elif key == ord('m'):
                self.switch_view_mode()
            elif key == ord('p'):
                self.toggle_preview()
        else:
            # Update FPS counter and print to console
            self.fps_counter.update(print_to_console=self.print_fps_to_console)
    
    def destroy_node(self):
        """Clean up resources before destroying the node"""
        self.running = False
        
        # Release camera resources
        if self.camera and self.camera.isOpened():
            self.camera.release()
        
        # Close any open windows
        if self.show_preview:
            cv2.destroyAllWindows()
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher_node = CameraPublisherNode()
    try:
        rclpy.spin(camera_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()