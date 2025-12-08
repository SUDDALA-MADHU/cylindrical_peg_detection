#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import PointStamped

class DualViewPegDetector(Node):
    def __init__(self):
        super().__init__('dual_view_peg_detector')
        self.bridge = CvBridge()
        self.depth_min_valid = 0.15
        self.depth_max_valid = 3.5
        self.blur_kernel = 5

        self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, 'camera/camera/aligned_depth_to_color/image_raw')
        self.info_sub = Subscriber(self, CameraInfo, '/camera/camera/color/camera_info')
        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub],
            queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        self.point_pub = self.create_publisher(PointStamped, 'peg_position', 10)
        self.latest_vis_image = None
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.get_logger().info("DualViewPegDetector started.")

    def callback(self, color_msg, depth_msg, info_msg):
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except Exception as e:
            self.get_logger().error(f"CV Bridge conversion failed: {e}")
            return

        if np.issubdtype(depth_raw.dtype, np.integer):
            depth = depth_raw.astype(np.float32) / 1000.0
        else:
            depth = depth_raw.astype(np.float32)

        # Preprocess image for circle detection
        color_blur = cv2.medianBlur(color, self.blur_kernel)
        gray = cv2.cvtColor(color_blur, cv2.COLOR_BGR2GRAY)

        # Hough circle detection with adaptive parameters
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=40,
                                   param1=100, param2=25, minRadius=12, maxRadius=140)

        if circles is None:
            self.latest_vis_image = color
            self.get_logger().info("No circles detected.")
            return

        # HSV mask for black color (changed from red)
        hsv = cv2.cvtColor(color_blur, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 60])
        black_mask = cv2.inRange(hsv, lower_black, upper_black)

        best_circle = None
        best_score = 0
        for circ in np.round(circles[0, :]).astype("int"):
            x, y, r = circ
            circle_mask = np.zeros_like(gray, dtype=np.uint8)
            cv2.circle(circle_mask, (x, y), r, 255, -1)
            black_amount = cv2.countNonZero(cv2.bitwise_and(black_mask, black_mask, mask=circle_mask))
            total_amount = cv2.countNonZero(circle_mask)
            black_ratio = black_amount / float(total_amount) if total_amount > 0 else 0
            # Only consider circles that are majority black
            if black_ratio > 0.55 and black_amount > 150:
                score = black_ratio * black_amount
                if score > best_score:
                    best_score = score
                    best_circle = (x, y, r)

        if best_circle is None:
            self.latest_vis_image = color
            self.get_logger().info("No valid black circle found.")
            return
        cx, cy, radius = best_circle

        # Robust depth sampling inside the black circle
        yy, xx = np.ogrid[0:depth.shape[0], 0:depth.shape[1]]
        mask = (xx - cx)**2 + (yy - cy)**2 <= radius*radius
        patch = depth[mask]
        patch = patch[(patch > self.depth_min_valid) & (patch < self.depth_max_valid) & np.isfinite(patch)]
        if len(patch) < 30:
            self.get_logger().warn("Not enough valid depth data in peg region.")
            return

        # Use percentiles for robust median
        lower, upper = np.percentile(patch, [15, 85])
        robust_patch = patch[(patch > lower) & (patch < upper)]
        if len(robust_patch) == 0:
            self.get_logger().warn("All depth values in region are outliers.")
            return
        depth_median = np.median(robust_patch)

        fx, fy = info_msg.k[0], info_msg.k[4]
        cx_cam, cy_cam = info_msg.k[2], info_msg.k[5]
        X = (cx - cx_cam) * depth_median / fx
        Y = (cy - cy_cam) * depth_median / fy
        Z = depth_median

        peg_point = PointStamped()
        peg_point.header = color_msg.header
        peg_point.point.x = float(X)
        peg_point.point.y = float(Y)
        peg_point.point.z = float(Z)
        self.point_pub.publish(peg_point)
        self.get_logger().info(f"Black peg at px=({cx},{cy}), r={radius} 3D=({X:.4f},{Y:.4f},{Z:.4f})m")

        vis = color.copy()
        cv2.circle(vis, (cx, cy), radius, (0, 255, 0), 2)
        cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(vis, f"Depth: {depth_median:.3f} m", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(vis, f"peg px:{cx},{cy} r:{radius}", (10, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1)
        self.latest_vis_image = vis
        cv2.imshow("Black Mask", black_mask)

    def timer_callback(self):
        if self.latest_vis_image is not None:
            cv2.imshow("Peg Detection", self.latest_vis_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DualViewPegDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down DualViewPegDetector")
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
