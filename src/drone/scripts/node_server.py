#!/usr/bin/env python3

import os
import json
import rospy
from flask import Flask, jsonify, send_from_directory, render_template_string, request
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger
import rospkg


# --- Setting ---
BASE_DIR = os.path.expanduser("~/mission_data")
app = Flask(__name__)
dispatch_pub = None

def init_ros_node():
    global dispatch_pub
    rospy.init_node('web_map_server', anonymous=True, disable_signals=True)
    dispatch_pub = rospy.Publisher('/mission/dispatch_req', NavSatFix, queue_size=10)
    rospy.loginfo("ROS Node & Publisher Initialized")

# --- HTML Load ---
rp = rospkg.RosPack()
pkg_path = rp.get_path('drone')
template_path = os.path.join(pkg_path, 'src', 'template.html')

with open(template_path, 'r', encoding='utf-8') as f:
    HTML_TEMPLATE = f.read()


# --- Flask Routes ---
@app.route('/')
def index(): 
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/fleet_status')
def get_fleet_status():
    try:
        rospy.wait_for_service('/fleet/get_status', timeout=0.5)
        srv = rospy.ServiceProxy('/fleet/get_status', Trigger)
        res = srv()
        return jsonify(json.loads(res.message) if res.success else [])
    except: 
        return jsonify([])

@app.route('/api/dispatch', methods=['POST'])
def dispatch_drone():
    data = request.json
    lat, lon = data.get('lat'), data.get('lon')
    if dispatch_pub and lat and lon:
        msg = NavSatFix()
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = 15.0
        msg.header.stamp = rospy.Time.now()
        dispatch_pub.publish(msg)
        return jsonify({'status': 'success', 'message': 'Command Transmit Complete'})
    return jsonify({'status': 'error', 'message': 'ROS Connection Failed'}), 500

@app.route('/api/missions')
def get_missions():
    if not os.path.exists(BASE_DIR): return jsonify([])
    try:
        dirs = sorted([d for d in os.listdir(BASE_DIR) if d.startswith("mission_") and os.path.isdir(os.path.join(BASE_DIR, d))], reverse=True)
        return jsonify(dirs)
    except: 
        return jsonify([])

@app.route('/api/data/<mission_id>')
def get_raw_data(mission_id):
    path = os.path.join(BASE_DIR, mission_id, "data_log.jsonl")
    data = []
    if os.path.exists(path):
        try:
            with open(path, 'r') as f:
                for line in f:
                    if line.strip(): 
                        data.append(json.loads(line))
        except: 
            pass
    return jsonify(data)

@app.route('/api/summary/<mission_id>')
def get_summary(mission_id):
    path = os.path.join(BASE_DIR, mission_id, "analysis", "summary.json")
    if os.path.exists(path):
        try:
            with open(path, 'r') as f:
                return jsonify(json.load(f))
        except: 
            pass
    return jsonify({"error": "No analysis"})

@app.route('/files/<mission_id>/<path:filename>')
def serve_files(mission_id, filename):
    return send_from_directory(os.path.join(BASE_DIR, mission_id), filename)

if __name__ == '__main__':
    try:
        init_ros_node()
    except Exception as e:
        rospy.loginfo(f"ROS Init Failed: {e}")
    
    rospy.loginfo(f"Smart Drone Command Center running at http://localhost:5000")
    app.run(host='0.0.0.0', port=5000, debug=False)