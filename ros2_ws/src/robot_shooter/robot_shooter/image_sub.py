from ast import Not
from enum import Flag
import time
from sympy import false
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64,Float64MultiArray,String
from cv_bridge import CvBridge
import cv2
import math
# from sift import MultiReferenceSIFTDetector
# from sift import create_reference_set
# import sift
from robot_shooter import sift

# MultiReferenceSIFTDetector


class ImageSub(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.pub_pan = self.create_publisher(Float64MultiArray,'/pan_controller/commands',10)
        self.pub_tilt = self.create_publisher(Float64MultiArray,'/tilt_controller/commands',10)
        self.delay = 0.5
        self.timer = self.create_timer(self.delay, self.pan_motion)
        # self.pub_tilt = self.create_publisher(Float64MultiArray,'/tilt_controller/commands',10)
        self.get_logger().info("Sending...")
        self.kp = 0.005
        self.br = CvBridge()
        self.motion = 0
        self.diff = (9999,9999)
        self.target_locked = False
        self.lock_timer = None
        self.lock_duration = 5.0  # 5 detik
        self.detected = False
        self.matches = 0

    def callback(self, msg: Image):
        # konversi ROS Image → OpenCV BGR
        
        detector = sift.MultiReferenceSIFTDetector()
        reference_images = sift.create_reference_set()
        for ref_img in reference_images:
            detector.add_reference(ref_img)

        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Capture video
        # cap = cv2.VideoCapture(1)
        detections = detector.detect(frame)

        if len(detections) == 0:
            self.get_logger().info("belum terdeteksi")
            self.diff = (9999,9999)
            self.detected = False
        else:
            self.detected = True
                # Draw detections
            for det in detections:
                # Draw bounding box
                corners = det['corners']
                self.matches = det['matches']
                cx, cy = ( (corners[0] + corners[2]) // 2, (corners[1] + corners[3]) // 2 )
                
                h, w = frame.shape[:2]
                ch, cw = h // 2, w // 2
                # ch,

                c_target = det['point']

                # dx = cw - c_target[0]
                # dy = ch - c_target[1] 
                dx = cw - cx
                dy = ch - cy 
                horizontal_fov = math.radians(60)  # konversi ke radian
                vertical_fov = math.radians(45)    # contoh saja
                
                self.diff = (dx,dy)
                angle_offset_pan = dx / (w / horizontal_fov)
                angle_offset_tilt = dy / (h / vertical_fov)

                pan_cmd = Float64MultiArray()
                tilt_cmd = Float64MultiArray()
                # pan_cmd.data = [angle_offset_pan]
                # tilt_cmd.data = [angle_offset_tilt]

                # self.pub_pan.publish(pan_cmd)
                # self.pub_tilt.publish(tilt_cmd)
                # self.diff = (dx,dy)
                
                
                cv2.rectangle(frame,pt1=(corners[0],corners[1]),pt2=(corners[2],corners[3]),color=(0,255,0),thickness=2)
                # Center detected
                cv2.circle(frame, (int(c_target[0]) ,int(c_target[1])), radius=5, color=(0, 0, 255), thickness=-1)
                    
                # Draw polygon around detected object
                conf_text = f"Center: {(c_target[0] , c_target[1])}"
                cv2.putText(frame, conf_text, (corners[0],corners[3]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        matches = str(self.matches) 
        h, w = frame.shape[:2]
        ch, cw = w // 2, h // 2
        cv2.circle(frame, (ch,cw), radius=5, color=(0, 255, 0), thickness=-1)
        c_text = f": {(ch,cw,self.diff)}"
        cv2.putText(frame, c_text, 
                    (ch,cw), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, (0, 255, 0), 2)
        cv2.putText(frame, matches, 
                    (100,80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    2, (0, 255, 0), 2)
        cv2.namedWindow("Multi-Reference SIFT Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Multi-Reference SIFT Detection", 640, 480)
        cv2.imshow('Multi-Reference SIFT Detection', frame)
        cv2.waitKey(1)
        # cv2.destroyAllWindows()


    def pan_motion(self):
        pan_cmd = Float64MultiArray()
        tilt_cmd = Float64MultiArray()
        log = String()
        pan_rad = math.radians(float(self.motion))
        tilt_rad = math.radians(float(-0.4))
        pan_cmd.data = [pan_rad]
        tilt_cmd.data = [tilt_rad]
        if not self.detected:
            tilt_cmd.data = [0.2]
            self.pub_pan.publish(pan_cmd)
            self.motion += 1
            return
        else:

            if abs(self.diff[0]) <= 3:
                self.get_logger().info('Target locker "%s"'% self.target_locked)
                self.get_logger().info("Holding position... %.2f seconds" % self.delay)
                self.pub_tilt.publish(tilt_cmd)
                time.sleep(5)
            else:
                self.delay = 0.5
                log.data = f"MOTION PAN: {self.motion:.2f}"
                self.get_logger().info('Publishing: "%s"' % log.data)
                self.pub_pan.publish(pan_cmd)
                self.motion += 0.5



def main(args=None):
    rclpy.init(args=args)
    node = ImageSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()