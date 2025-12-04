#!/usr/bin/env python3
# yolo_mission_logger.py

import rospy
import os
import json
import threading
import cv2
import numpy as np
from datetime import datetime
from sensor_msgs.msg import Image, NavSatFix
from mavros_msgs.msg import State
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge

# --- YOLO-World Imports ---
from ultralytics import YOLOWorld

# --- Config ---
BASE_DIR = rospy.get_param("BASE_DIR", "~/mission_data")
IMAGE_TOPIC = rospy.get_param("IMAGE_TOPIC")
GPS_TOPIC = rospy.get_param("GPS_TOPIC")
STATE_TOPIC = rospy.get_param("STATE_TOPIC")
BBOX_TOPIC = rospy.get_param("BBOX_TOPIC")
VIS_TOPIC = rospy.get_param("VIS_TOPIC")

MODEL_NAME = rospy.get_param("MODEL_NAME", "yolov8x-worldv2.pt")
CUSTOM_CLASSES = rospy.get_param("CUSTOM_CLASSES", [])
CONF_THRESHOLD = rospy.get_param("CONF_THRESHOLD", 0.2)
MAX_BBOX_RATIO = rospy.get_param("MAX_BBOX_RATIO", 0.05)




class YoloMissionLogger:
    def __init__(self):
        rospy.init_node("yolo_mission_logger")
        
        if not os.path.exists(BASE_DIR):
            os.makedirs(BASE_DIR)

        # --------------------------------------------------------
        # YOLO-World Model Load
        # --------------------------------------------------------
        self.device = 0 # GPU: 0, CPU: 'cpu'
        try:
            rospy.loginfo(f"Loading YOLO-World model: {MODEL_NAME}...")
            self.model = YOLOWorld(MODEL_NAME)
            
            self.model.set_classes(CUSTOM_CLASSES)
            
            rospy.loginfo(f"Model Loaded. Target Classes: {CUSTOM_CLASSES}")
        except Exception as e:
            rospy.logerr(f"Model Load Failed: {e}")
            return

        self.bridge = CvBridge()
        self.latest_img = None
        self.latest_gps = None
        self.current_state = None
        
        # Mission Manage Variables
        self.is_mission_active = False
        self.current_mission_dir = None
        self.current_log_file = None
        
        self.bbox_pub = rospy.Publisher(BBOX_TOPIC, Detection2DArray, queue_size=1)
        self.vis_pub = rospy.Publisher(VIS_TOPIC, Image, queue_size=1)

        rospy.Subscriber(IMAGE_TOPIC, Image, self.img_cb, queue_size=1)
        rospy.Subscriber(GPS_TOPIC, NavSatFix, self.gps_cb, queue_size=1)
        rospy.Subscriber(STATE_TOPIC, State, self.state_cb, queue_size=1)

        self.loop_thread = threading.Thread(target=self.inference_loop, daemon=True)
        self.loop_thread.start()

    def img_cb(self, msg): self.latest_img = msg
    def gps_cb(self, msg): self.latest_gps = msg
    
    def state_cb(self, msg):
        # MAVROS State Callback (Mission Auto Start/End Trigger)
        prev_state = self.current_state
        self.current_state = msg
        
        is_armed = msg.armed
        is_mission = (msg.mode == "AUTO.MISSION")
        
        if is_armed and is_mission:
            if not self.is_mission_active:
                self.start_new_mission()
        else:
            if self.is_mission_active:
                self.end_mission()

    def start_new_mission(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        dir_name = f"mission_{timestamp}"
        self.current_mission_dir = os.path.join(BASE_DIR, dir_name)
        
        os.makedirs(self.current_mission_dir, exist_ok=True)
        os.makedirs(os.path.join(self.current_mission_dir, "images"), exist_ok=True)
        
        self.current_log_file = os.path.join(self.current_mission_dir, "data_log.jsonl")
        self.is_mission_active = True
        rospy.loginfo(f"Mission Started! Saving to: {dir_name}")

    def end_mission(self):
        self.is_mission_active = False
        self.current_mission_dir = None
        self.current_log_file = None
        rospy.loginfo("Mission Ended. Logging stopped.")

    def inference_loop(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            if not self.is_mission_active or self.latest_img is None:
                rate.sleep()
                continue
            
            try:
                img_msg = self.latest_img
                gps_msg = self.latest_gps
                self.latest_img = None # 큐 비우기

                # ROS Image -> OpenCV Image
                cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
                img_h, img_w = cv_img.shape[:2]
                timestamp = img_msg.header.stamp.to_sec()

                # --------------------------------------------------------
                # [수정됨] YOLO-World 추론 (Standard Inference)
                # --------------------------------------------------------
                # set_classes를 했으므로 해당 클래스들만 탐지됨
                results = self.model.predict(
                    source=cv_img,
                    conf=CONF_THRESHOLD,
                    device=self.device,
                    verbose=False, # 로그 출력 끔
                    imgsz=640
                )
                
                detections = []
                vis_img = cv_img.copy()
                has_person = False

                # Ultralytics 결과 파싱
                # results[0]은 첫 번째 이미지의 결과 (배치 사이즈 1이므로)
                result = results[0]
                
                # GPU 텐서를 CPU numpy로 변환
                boxes = result.boxes.cpu().numpy()

                for box in boxes:
                    # xyxy: [x1, y1, x2, y2]
                    x1, y1, x2, y2 = box.xyxy[0].astype(int)
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    class_name = result.names[cls_id] # 'person', 'human' 등

                    # 1. 크기 필터링 (너무 큰 박스는 노이즈일 확률 높음)
                    box_area = (x2 - x1) * (y2 - y1)
                    img_area = img_h * img_w
                    if (box_area / img_area) > MAX_BBOX_RATIO:
                        continue

                    # 2. 시각화 (빨간색 박스)
                    cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    label = f"{class_name} {conf:.2f}"
                    cv2.putText(vis_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    # 3. 데이터 저장용 리스트 추가
                    detections.append({
                        "class_name": class_name, # YOLO-World가 매핑한 텍스트
                        "score": conf,
                        "box": [int(x1), int(y1), int(x2), int(y2)] # int 변환 필수 (JSON 직렬화)
                    })
                    has_person = True

                # 시각화 토픽 발행
                self.vis_pub.publish(self.bridge.cv2_to_imgmsg(vis_img, "bgr8"))

                # 데이터 파일 저장 (미션 중일 때만)
                if self.is_mission_active and self.current_log_file:
                    self.save_data(timestamp, vis_img, gps_msg, detections)

            except Exception as e:
                rospy.logerr(f"Inference Error: {e}")

    def save_data(self, timestamp, img, gps, dets):
        if not self.current_mission_dir: return
        
        img_name = f"{timestamp:.3f}.jpg"
        img_path = os.path.join(self.current_mission_dir, "images", img_name)
        
        # 이미지 저장
        cv2.imwrite(img_path, img)

        # 로그 저장
        log_entry = {
            "timestamp": timestamp,
            "gps": {"lat": gps.latitude, "lon": gps.longitude} if gps else {"lat":0,"lon":0},
            "detections": dets,
            "image_path": f"images/{img_name}" # 상대 경로
        }
        
        with open(self.current_log_file, "a") as f:
            f.write(json.dumps(log_entry) + "\n")

if __name__ == "__main__":
    YoloMissionLogger()
    rospy.spin()