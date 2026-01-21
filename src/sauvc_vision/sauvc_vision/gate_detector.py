#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # Using Point to send (x_error, y_error, confidence)
from cv_bridge import CvBridge
import cv2
import numpy as np

class GateDetectorNode(Node):
    def __init__(self):
        super().__init__('gate_detector')

        # --- TUNING PARAMETERS ---
        self.TARGET_RATIO = 1.5  # Based on your tuned value
        self.MIN_AREA = 500
        
        # --- ROS COMMUNICATION ---
        # 1. Subscriber: Listens to the camera (Change topic name if needed!)
        self.subscription = self.create_subscription(
            Image, 
            '/image_raw', 
            self.image_callback, 
            10)
            
        # 2. Publisher: Sends Control Data (X offset, Y offset, Confidence)
        # We use Point for simplicity: x=norm_x, y=norm_y, z=confidence
        self.publisher_ = self.create_publisher(Point, '/gate/data', 10)
        
        # 3. Publisher: Sends Debug Image (with drawings) to view in Rqt
        self.debug_pub = self.create_publisher(Image, '/gate/debug', 10)

        self.bridge = CvBridge()
        self.get_logger().info("✅ Gate Detector Node Started.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image -> OpenCV Image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # =========================================
        # YOUR ALGORITHM STARTS HERE
        # =========================================
        
        # 1. Pre-Processing (CLAHE + Blur + Canny)
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        l = clahe.apply(l)
        lab = cv2.merge((l,a,b))
        corr = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        blur = cv2.bilateralFilter(corr, 9, 75, 75)
        edges = cv2.Canny(blur, 50, 150)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        edges_closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)

        # 2. Find Contours
        contours, _ = cv2.findContours(edges_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 3. Find Best Candidate
        H, W = frame.shape[:2]
        img_cx, img_cy = W//2, H//2
        
        best_candidate = None
        max_score = 0
        final_vals = (0.0, 0.0, 0.0) # x, y, conf
        detected = False

        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.MIN_AREA:
                x, y, w, h = cv2.boundingRect(cnt)
                candidates.append((area, x, y, w, h))

        if candidates:
            # Pick largest by area
            best = max(candidates, key=lambda x: x[0])
            area, x, y, w, h = best
            
            # Geometry
            gate_cx = x + w // 2
            gate_cy = y + h // 2
            
            norm_x = (gate_cx - img_cx) / (W / 2)
            norm_y = (img_cy - gate_cy) / (H / 2) # ROS convention: Up is +z in 3D, but usually -y in 2D image

            # Confidence Logic
            measured_ratio = float(w) / h
            ratio_diff = abs(measured_ratio - self.TARGET_RATIO)
            ratio_score = max(0, 1.0 - (ratio_diff / 0.5))
            confidence = (ratio_score ** 2)

            if confidence > 0.5: # Only publish if somewhat confident
                detected = True
                final_vals = (norm_x, norm_y, confidence)
                
                # Draw Debug Info
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(frame, (gate_cx, gate_cy), 5, (0, 0, 255), -1)
                cv2.line(frame, (img_cx, img_cy), (gate_cx, gate_cy), (0, 255, 255), 2)
                label = f"Conf: {confidence:.2f}"
                cv2.putText(frame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        # =========================================
        # PUBLISH RESULTS
        # =========================================
        
        # 1. Data Message
        msg_point = Point()
        msg_point.x = float(final_vals[0])  # Offset X (Turn Left/Right)
        msg_point.y = float(final_vals[1])  # Offset Y (Go Up/Down)
        msg_point.z = float(final_vals[2])  # Confidence (0.0 to 1.0)
        self.publisher_.publish(msg_point)

        # 2. Debug Image
        if self.debug_pub.get_subscription_count() > 0:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.debug_pub.publish(debug_msg)

        if detected:
            self.get_logger().info(f"Gate Detected! X:{final_vals[0]:.2f} Conf:{final_vals[2]:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = GateDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
