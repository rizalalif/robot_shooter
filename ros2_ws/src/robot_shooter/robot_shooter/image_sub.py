import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64,Float64MultiArray
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
        self.kp = 0.005
        self.br = CvBridge()

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
            
            # Draw detections
        for det in detections:
            # Draw bounding box
            corners = det['corners']
            cx, cy = ( (corners[0] + corners[2]) // 2, (corners[1] + corners[3]) // 2 )
            
            h, w = frame.shape[:2]
            ch, cw = w // 2, h // 2

            dx = cx - cw
            dy = cy - ch
            horizontal_fov = math.radians(60)  # konversi ke radian
            vertical_fov = math.radians(45)    # contoh saja
            

            angle_offset_pan = dx / (w / horizontal_fov)
            angle_offset_tilt = dy / (h / vertical_fov)

            pan_cmd = Float64MultiArray()
            tilt_cmd = Float64MultiArray()
            pan_cmd.data = [angle_offset_pan]
            tilt_cmd.data = [angle_offset_tilt]

            self.pub_pan.publish(pan_cmd)
            self.pub_tilt.publish(tilt_cmd)
            
            
            cv2.rectangle(frame,pt1=(corners[0],corners[1]),pt2=(corners[2],corners[3]),color=(0,255,0),thickness=2)
            # Center detected
            cv2.circle(frame, (cx, cy), radius=5, color=(0, 0, 255), thickness=-1)
                
                # Draw polygon around detected object
            # cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
                
                # Add confidence score
            # conf_text = f"Conf: {det['confidence']:.2f}"
            conf_text = f"Center: {(cx,cy)}"
            cv2.putText(frame, conf_text, (corners[0],corners[3]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        h, w = frame.shape[:2]
        ch, cw = w // 2, h // 2
        cv2.circle(frame, (ch,cw), radius=5, color=(0, 255, 0), thickness=-1)
        cv2.namedWindow("Multi-Reference SIFT Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Multi-Reference SIFT Detection", 1280, 720)
        cv2.imshow('Multi-Reference SIFT Detection', frame)
        cv2.waitKey(1)
        # cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()