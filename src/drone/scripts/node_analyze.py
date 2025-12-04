#!/usr/bin/env python3

import rospy
import os
import json
import glob
import time
import cv2
import numpy as np
import pandas as pd
from sklearn.cluster import DBSCAN
from mavros_msgs.msg import State

BASE_DIR = os.path.expanduser("~/mission_data")

# --- Setting ---
MIN_SAMPLES = rospy.get_param("MIN_SAMPLES")
EPS_METERS = rospy.get_param("EPS_METERS")
METER_PER_DEG = 111000 

class MissionAnalyzer:
    def __init__(self):
        rospy.init_node("mission_analyzer")
        self.current_state = None
        self.last_processed_mission = None
        self.is_analyzing = False

        rospy.Subscriber("/observe/mavros/state", State, self.state_cb)
        
        rospy.loginfo("Mission Analyzer Standing By...")
        self.check_loop()

    def state_cb(self, msg):
        self.current_state = msg

    def check_loop(self):
        rate = rospy.Rate(0.2)
        while not rospy.is_shutdown():
            all_missions = sorted(glob.glob(os.path.join(BASE_DIR, "mission_*")))
            if not all_missions:
                rate.sleep()
                continue
                
            latest_mission = all_missions[-1]
            
            if latest_mission == self.last_processed_mission:
                rate.sleep()
                continue

            log_path = os.path.join(latest_mission, "data_log.jsonl")
            if not os.path.exists(log_path):
                rate.sleep()
                continue
                
            mtime = os.path.getmtime(log_path)
            if (time.time() - mtime) > 10.0:
                rospy.loginfo(f"Analyzing Mission: {os.path.basename(latest_mission)}...")
                self.analyze_mission(latest_mission)
                self.last_processed_mission = latest_mission

            rate.sleep()

    def analyze_mission(self, mission_path):
        data = []
        log_file = os.path.join(mission_path, "data_log.jsonl")
        
        with open(log_file, 'r') as f:
            for line in f:
                try:
                    entry = json.loads(line)
                    if len(entry['detections']) > 0:
                        data.append(entry)
                except: 
                    pass

        if len(data) == 0:
            rospy.logwarn("No detections found in this mission.")
            return

        df = pd.DataFrame(data)
        
        coords = df['gps'].apply(lambda x: [x['lat'], x['lon']]).tolist()
        coords = np.array(coords)

        ref_lat, ref_lon = coords[0]
        x_diff = (coords[:, 1] - ref_lon) * METER_PER_DEG * np.cos(np.radians(ref_lat))
        y_diff = (coords[:, 0] - ref_lat) * METER_PER_DEG
        
        X = np.column_stack((x_diff, y_diff))

        db = DBSCAN(eps=EPS_METERS, min_samples=MIN_SAMPLES).fit(X)
        labels = db.labels_

        df['cluster'] = labels
        
        analysis_dir = os.path.join(mission_path, "analysis")
        os.makedirs(analysis_dir, exist_ok=True)

        summary_results = []

        unique_labels = set(labels)
        for k in unique_labels:
            if k == -1: 
                continue

            cluster_data = df[df['cluster'] == k]
            
            avg_lat = cluster_data['gps'].apply(lambda x: x['lat']).mean()
            avg_lon = cluster_data['gps'].apply(lambda x: x['lon']).mean()
            
            video_filename = f"event_cluster_{k}.mp4"
            video_path = os.path.join(analysis_dir, video_filename)
            
            success = self.create_video(cluster_data, mission_path, video_path)
            
            if not success:
                rospy.logwarn(f"Failed to create video for cluster {k}")
                continue

            summary_results.append({
                "cluster_id": int(k),
                "center_gps": {"lat": avg_lat, "lon": avg_lon},
                "start_time": cluster_data.iloc[0]['timestamp'],
                "end_time": cluster_data.iloc[-1]['timestamp'],
                "count": int(len(cluster_data)),
                "video_path": f"analysis/{video_filename}"
            })

        with open(os.path.join(analysis_dir, "summary.json"), "w") as f:
            json.dump(summary_results, f, indent=4)
        
        rospy.loginfo(f"Analysis Complete. Found {len(summary_results)} events.")

    def create_video(self, cluster_df, mission_path, output_path):
        if len(cluster_df) == 0:
            return False

        first_row = cluster_df.iloc[0]
        first_img_path = os.path.join(mission_path, first_row['image_path'])
        first_img = cv2.imread(first_img_path)
        if first_img is None:
            return False
            
        h, w, _ = first_img.shape

        fourcc = cv2.VideoWriter_fourcc(*'avc1')
        out = cv2.VideoWriter(output_path, fourcc, 5.0, (w, h))
        
        if not out.isOpened():
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(output_path, fourcc, 5.0, (w, h))
            if not out.isOpened():
                return False

        object_tracks = {}  # {object_id: [(x, y), ...]}
        next_id = 1
        prev_centers = {}  # {object_id: (cx, cy)}
        
        frame_count = 0
        for idx, row in cluster_df.iterrows():
            img_path = os.path.join(mission_path, row['image_path'])
            frame = cv2.imread(img_path)
            if frame is None:
                continue
            
            detections = row.get('detections', [])
            current_centers = {}
            
            for det in detections:
                bbox = det.get('bbox', [])
                if len(bbox) != 4:
                    continue
                    
                x1, y1, x2, y2 = map(int, bbox)
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                
                matched_id = None
                min_dist = float('inf')
                
                for obj_id, prev_center in prev_centers.items():
                    dist = np.sqrt((cx - prev_center[0])**2 + (cy - prev_center[1])**2)
                    if dist < 100 and dist < min_dist:
                        min_dist = dist
                        matched_id = obj_id
                
                if matched_id is None:
                    matched_id = next_id
                    next_id += 1
                    object_tracks[matched_id] = []
                
                object_tracks[matched_id].append((cx, cy))
                current_centers[matched_id] = (cx, cy)
                
                color = self.get_color_for_id(matched_id)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                
                class_name = det.get('class_name', det.get('class_id', 'Person'))
                score = det.get('score', 0.0)
                label = f"ID:{matched_id} {class_name} {score:.2f}"
                
                (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(frame, (x1, y1 - text_h - 10), (x1 + text_w, y1), color, -1)
                cv2.putText(frame, label, (x1, y1 - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            for obj_id, track in object_tracks.items():
                if len(track) < 2:
                    continue
                color = self.get_color_for_id(obj_id)
                for i in range(len(track) - 1):
                    cv2.line(frame, track[i], track[i+1], color, 2)
            
            gps_info = row.get('gps', {})
            timestamp = row.get('timestamp', 0)
            time_str = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(timestamp))
            
            info_text = f"Time: {time_str} | GPS: {gps_info.get('lat', 0):.6f}, {gps_info.get('lon', 0):.6f}"
            cv2.rectangle(frame, (10, 10), (w - 10, 50), (0, 0, 0), -1)
            cv2.putText(frame, info_text, (15, 35), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            frame_text = f"Frame: {frame_count + 1}/{len(cluster_df)}"
            cv2.putText(frame, frame_text, (15, h - 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            out.write(frame)
            prev_centers = current_centers
            frame_count += 1
        
        out.release()
        
        if frame_count == 0 or not os.path.exists(output_path):
            rospy.logerr(f"Video creation failed: {output_path}")
            return False
            
        file_size = os.path.getsize(output_path)
        if file_size < 1000:
            rospy.logerr(f"Video file too small: {file_size} bytes")
            return False
            
        rospy.loginfo(f"Video created: {frame_count} frames, {file_size} bytes")
        
        try:
            import subprocess
            temp_path = output_path + ".temp.mp4"
            os.rename(output_path, temp_path)
            
            cmd = [
                'ffmpeg', '-y', '-i', temp_path,
                '-vcodec', 'libx264',
                '-pix_fmt', 'yuv420p',
                '-movflags', '+faststart',
                output_path
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                os.remove(temp_path)
                rospy.loginfo(f"Video re-encoded with ffmpeg for better compatibility")
            else:
                os.rename(temp_path, output_path)
                rospy.logwarn("ffmpeg re-encoding failed, using original video")
                
        except Exception as e:
            rospy.logwarn(f"ffmpeg not available or failed: {e}")
        
        return True
    
    def get_color_for_id(self, obj_id):
        colors = [
            (255, 0, 0),    # Red
            (0, 255, 0),    # Green
            (0, 0, 255),    # Blue
            (255, 255, 0),  # Yellow
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Sian
            (255, 128, 0),  # Orange
            (128, 0, 255),  # Purple
        ]
        return colors[obj_id % len(colors)]

if __name__ == "__main__":
    MissionAnalyzer()