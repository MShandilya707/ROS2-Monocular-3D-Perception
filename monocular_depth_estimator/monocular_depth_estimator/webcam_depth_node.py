import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
from ultralytics import YOLO

class WebcamDepthNode(Node):
    def __init__(self):
        super().__init__('webcam_depth_node')
        
        self.get_logger().info("Initializing Webcam Depth Node (CPU)...")
        
        # Publishers
        self.depth_pub = self.create_publisher(Image, 'depth_image', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.rgb_pub = self.create_publisher(Image, 'image_raw', 10)
        
        self.bridge = CvBridge()
        
        # 1. Initialize Static Image
        image_path = '/home/mshandilya707/.gemini/antigravity-ide/brain/579f0690-04b7-46cc-a716-8d32e7c5f482/test_image_1782989505969.png'
        self.static_image = cv2.imread(image_path)
        if self.static_image is None:
            self.get_logger().error(f"Could not open image file {image_path}")
            return
            
        self.height, self.width, _ = self.static_image.shape
        self.get_logger().info(f"Image opened at resolution: {self.width}x{self.height}")
        
        # 2. Load PyTorch MiDaS Model (CPU optimized)
        self.get_logger().info("Downloading/Loading MiDaS Small model...")
        
        # Trust the backbone repository that MiDaS depends on
        torch.hub.list("rwightman/gen-efficientnet-pytorch", trust_repo=True)
        
        model_type = "MiDaS_small"
        self.midas = torch.hub.load("intel-isl/MiDaS", model_type, trust_repo=True)
        self.device = torch.device("cpu")
        self.midas.to(self.device)
        self.midas.eval()
        
        midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
        self.transform = midas_transforms.small_transform

        # 3. Load YOLOv8 Segmentation Model
        self.get_logger().info("Downloading/Loading YOLOv8 Nano Segmentation model...")
        self.yolo_model = YOLO('yolov8n-seg.pt')
        self.get_logger().info("YOLOv8 model loaded successfully!")
        
        self.get_logger().info("Model loaded successfully! Starting inference timer.")
        
        # 3. Create a timer for the processing loop (e.g., 10 Hz)
        # MiDaS small on CPU takes roughly 50-100ms, so 10 Hz (0.1s) is a good target.
        self.timer = self.create_timer(0.1, self.timer_callback)

    def generate_camera_info(self, stamp):
        """Generates a synthetic CameraInfo message."""
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = 'camera_depth_optical_frame'
        info.width = self.width
        info.height = self.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Approximate Focal Length (assuming typical webcam FOV of ~60 degrees)
        # fx = fy = width / (2 * tan(HFOV/2))
        fx = self.width / 1.15
        fy = fx
        cx = self.width / 2.0
        cy = self.height / 2.0
        
        # Intrinsic matrix [fx, 0, cx; 0, fy, cy; 0, 0, 1]
        info.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
                  
        # Projection matrix
        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
                  
        return info

    def timer_callback(self):
        frame = self.static_image.copy()

        # Run YOLO inference and get the annotated frame with masks and boxes
        results = self.yolo_model(frame, verbose=False)
        annotated_frame = results[0].plot()

        # Record the exact ROS time for synchronization
        now = self.get_clock().now().to_msg()
        
        # Convert BGR (OpenCV) to RGB (MiDaS)
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Apply transforms and run inference
        input_batch = self.transform(img).to(self.device)
        
        with torch.no_grad():
            prediction = self.midas(input_batch)
            
            # Resize the prediction to match the original image resolution
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=img.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        # MiDaS outputs disparity (inverse depth). 
        # We need to invert it back to get pseudo-depth for our math node.
        # depth = 1 / (disparity + tiny_number_to_prevent_div_by_zero)
        # We also apply an arbitrary scaling factor to make it look decent in RViz.
        output = prediction.cpu().numpy()
        
        # We MUST clip the depth to a reasonable range (e.g., 0.1m to 20m) 
        # Otherwise, PCL's VoxelGrid will crash with "Integer indices would overflow" 
        # if the AI hallucinates a point at infinity!
        depth_map = np.clip(100.0 / (output + 0.0001), 0.1, 20.0)
        
        # The C++ node expects 32-bit floats
        depth_map = depth_map.astype(np.float32)

        # 1. Publish Depth Image
        depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding="32FC1")
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        self.depth_pub.publish(depth_msg)

        # 2. Publish Camera Info
        info_msg = self.generate_camera_info(now)
        self.info_pub.publish(info_msg)

        # 3. Publish Annotated image (for C++ node to extract colors and RViz visualization)
        rgb_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        rgb_msg.header.stamp = now
        rgb_msg.header.frame_id = 'camera_depth_optical_frame'
        self.rgb_pub.publish(rgb_msg)

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
