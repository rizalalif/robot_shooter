import cv2
import cv2.xfeatures2d
import numpy as np

class MultiReferenceSIFTDetector:
    def __init__(self):
        # Inisialisasi SIFT detector
        self.sift = cv2.xfeatures2d.SIFT_create()
        # FLANN matcher
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        
        # Simpan keypoints dan descriptors dari reference images
        self.ref_keypoints = []
        self.ref_descriptors = []
        
    def add_reference(self, image):
        """Menambahkan reference image"""
        if isinstance(image, str):
            image = cv2.imread(image)
        gray_ref = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        resized_ref = cv2.resize(gray_ref,(0, 0), fx = 0.01, fy = 0.01)
        # clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(8, 8))
        # claheImg = clahe.apply(gray)
        # claheImg = cv2.GaussianBlur(claheImg, (5, 5), 0)
        
        # Detect keypoints dan compute descriptors
        kp, des = self.sift.detectAndCompute(gray_ref, None)
        
        if des is not None:
            self.ref_keypoints.append(kp)
            self.ref_descriptors.append(des)
            return True
        return False
        
    def detect(self, frame, min_matches=4, ratio_thresh=0.62):
        """Detect shuttlecock dalam frame"""
        # Convert frame ke grayscale
        gray_input = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(8, 8))
        # claheImg = clahe.apply(gray)
        # claheImg = cv2.GaussianBlur(claheImg, (5, 5), 0)
        
        # Detect keypoints dan compute descriptors untuk frame
        kp_frame, des_frame = self.sift.detectAndCompute(gray_input, None)
        
        if des_frame is None:
            return []
            
        all_detections = []
        
        # Check matches dengan setiap reference image
        for i, ref_des in enumerate(self.ref_descriptors):
            # Find matches menggunakan FLANN matcher
            matches = self.flann.knnMatch(ref_des, des_frame, k=2)
            
            # Apply ratio test
            good_matches = []
            for m, n in matches:
                if m.distance < ratio_thresh * n.distance:
                    good_matches.append(m)
            
            if len(good_matches) >= min_matches:
                # Get matched keypoints
                ref_pts = np.float32([self.ref_keypoints[i][m.queryIdx].pt 
                                    for m in good_matches]).reshape(-1, 1, 2)
                frame_pts = np.float32([kp_frame[m.trainIdx].pt 
                                      for m in good_matches]).reshape(-1, 1, 2)
                
                pts = np.float32([kp_frame[m.trainIdx].pt for m in good_matches])
                # Calculate homography
                H, mask = cv2.findHomography(ref_pts, frame_pts, cv2.RANSAC, 5.0)
                
                if H is None:
                    continue
                #     Get corners dari reference image
                h, w = gray_input.shape
                corners = np.float32([[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)
                    
                # Transform corners ke frame
                transformed_corners = cv2.perspectiveTransform(corners, H)
                    


                # Gambar kotaks
                pts = transformed_corners.reshape(-1, 2)
                x_min, y_min = pts.min(axis=0).astype(int)
                x_max, y_max = pts.max(axis=0).astype(int)
                all_detections.append({
                        'corners': [x_min,y_min,x_max,y_max],
                        'matches': len(good_matches),
                        'confidence': len(good_matches) / len(matches)
                })
        return all_detections

def create_reference_set():
    """Fungsi untuk membuat set referensi dari berbagai sudut pandang"""
    references = []
    
    # Bisa ditambahkan multiple reference images:
    # 1. Shuttlecock tampak depan
    # 2. Shuttlecock tampak samping
    # 3. Shuttlecock dengan berbagai rotasi
    # 4. Shuttlecock dengan berbagai pencahayaan
    # 5. Shuttlecock dengan berbagai skala

    reference_paths = [
        '/root/ros2_ws/src/robot_shooter/robot_shooter/img/sengkuni1.jpg',
        '/root/ros2_ws/src/robot_shooter/robot_shooter/img/durna.jpg',
        # 'data/burisrawa.jpg',
        # 'data/salep3.jpg',
        # 'data/salep4.jpg',
        # 'data/salep5.jpg',
        # 'data/salep6.jpg',
        # Tambahkan path gambar lainnya sesuai kebutuhan
    ]
    
    # Load semua gambar referensi
    for path in reference_paths:
        try:
            img = cv2.imread(path)
            img = cv2.resize(img,(480,640))

            if img is not None:
                references.append(img)
                (h,w) = img.shape[:2]
                print(str(h) + ","+str(w)) 
            else:
                print(f"Warning: Tidak bisa membaca gambar dari {path}")
        except Exception as e:
            print(f"Error loading image {path}: {str(e)}")
    
    return references
    

# Penggunaan
detector = MultiReferenceSIFTDetector()
# ======================================================================================================================
# Tambahkan reference images
reference_images = create_reference_set()
for ref_img in reference_images:
    detector.add_reference(ref_img)

# Capture video
cap = cv2.VideoCapture("http://192.168.18.27:4747/video")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Detect shuttlecock
    detections = detector.detect(frame)
    
    # Draw detections
    for det in detections:
        # Draw bounding box
        corners = det['corners']
        # corners = corners.astype(int)
        cx, cy = ( (corners[0] + corners[2]) // 2, (corners[1] + corners[3]) // 2 )

          
        # Draw polygon around detected object
        # cv2.polylines(frame, [corners], True, (0, 255, 0), 2)

        # draw rect
        cv2.rectangle(frame,pt1=(corners[0],corners[1]),pt2=(corners[2],corners[3]),color=(0,255,0),thickness=2)
         # Hitung titik tengah kotak
        # Gambar titik merah di tengah
        cv2.circle(frame, (cx, cy), radius=5, color=(0, 0, 255), thickness=-1)
        # Add confidence score
        conf_text = f"Conf: {det['confidence']:.2f}"
        cv2.putText(frame, conf_text, 
                    (corners[0],corners[3]), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, (0, 255, 0), 2)
    
    h, w = frame.shape[:2]
    ch, cw = w // 2, h // 2
    cv2.circle(frame, (ch,cw), radius=5, color=(0, 255, 0), thickness=-1)

        
    cv2.namedWindow('Multi-Reference SIFT Detection', cv2.WINDOW_NORMAL)

    # Set ukuran window (pixel)
    cv2.resizeWindow('Multi-Reference SIFT Detection', 640, 480)
    cv2.imshow('Multi-Reference SIFT Detection', frame)
    
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()