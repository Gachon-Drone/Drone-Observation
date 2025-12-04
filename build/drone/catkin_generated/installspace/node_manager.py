#!/usr/bin/env python3
import rospy
import math
import json
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, BatteryState
from mavros_msgs.msg import State, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import Trigger, TriggerResponse

DRONE_IDS = rospy.get_param("DRONE_IDS", [])

class DroneAgent:
    # ... (기존 __init__, state_cb, gps_cb, control_loop, check_arrival 등 동일) ...
    # ... (변경 사항 없음) ...
    
    def __init__(self, drone_id):
        # ... 기존 코드 동일 ...
        self.drone_id = drone_id
        self.is_busy = False
        self.current_state = State()
        self.current_gps = None
        self.battery_voltage = 0.0
        
        self.target_coords = None
        self.target_msg = None 

        self.ns = f"/{drone_id}/mavros"
        
        rospy.Subscriber(f"{self.ns}/state", State, self.state_cb)
        rospy.Subscriber(f"{self.ns}/global_position/global", NavSatFix, self.gps_cb)
        rospy.Subscriber(f"{self.ns}/battery", BatteryState, self.bat_cb)
        
        self.pub_global = rospy.Publisher(f"{self.ns}/setpoint_raw/global", GlobalPositionTarget, queue_size=10)
        
        self.srv_arming = rospy.ServiceProxy(f"{self.ns}/cmd/arming", CommandBool)
        self.srv_mode = rospy.ServiceProxy(f"{self.ns}/set_mode", SetMode)

        rospy.Timer(rospy.Duration(0.05), self.control_loop)

    def state_cb(self, msg): self.current_state = msg
    def gps_cb(self, msg): self.current_gps = msg
    def bat_cb(self, msg): self.battery_voltage = msg.voltage

    def control_loop(self, event):
        if self.target_msg is not None and self.is_busy:
            self.target_msg.header.stamp = rospy.Time.now()
            self.pub_global.publish(self.target_msg)
            self.check_arrival()

    def check_arrival(self):
        if not self.current_gps or not self.target_coords: return
        dist = self.distance_to(self.target_coords[0], self.target_coords[1])
        if dist < 3.0:
            rospy.loginfo(f"[{self.drone_id}] Arrived! Hovering.")
            self.set_mode("AUTO.LOITER")
            self.is_busy = False
            self.target_msg = None
            self.target_coords = None

    def distance_to(self, target_lat, target_lon):
        if not self.current_gps: return float('inf')
        R = 6371000
        phi1 = math.radians(self.current_gps.latitude)
        phi2 = math.radians(target_lat)
        dphi = math.radians(target_lat - self.current_gps.latitude)
        dlambda = math.radians(target_lon - self.current_gps.longitude)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2) * math.sin(dlambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    def is_available(self):
        return self.current_state.connected and not self.is_busy

    def dispatch(self, lat, lon, alt):
        rospy.loginfo(f"[{self.drone_id}] Dispatch -> {lat:.5f}, {lon:.5f}")
        self.is_busy = True
        self.target_coords = (lat, lon, alt)
        
        target = GlobalPositionTarget()
        target.header.stamp = rospy.Time.now()
        target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        target.type_mask = 0b110111111000 
        target.latitude = lat
        target.longitude = lon
        target.altitude = alt 
        self.target_msg = target
        
        rospy.sleep(0.2)
        if not self.current_state.armed: self.set_arming(True)
        rospy.sleep(2.0)
        if self.current_state.mode != "OFFBOARD": self.set_mode("OFFBOARD")

    def set_mode(self, mode):
        try: self.srv_mode(custom_mode=mode)
        except: pass
    def set_arming(self, arm):
        try: self.srv_arming(value=arm)
        except: pass

    def get_status_dict(self):
        lat = self.current_gps.latitude if self.current_gps else 0.0
        lon = self.current_gps.longitude if self.current_gps else 0.0
        return {
            "id": self.drone_id,
            "connected": self.current_state.connected,
            "mode": self.current_state.mode,
            "battery": round(self.battery_voltage, 1),
            "is_busy": self.is_busy,
            "location": {"lat": lat, "lon": lon}
        }

class FleetManager:
    def __init__(self):
        rospy.init_node('fleet_manager')
        self.agents = [DroneAgent(did) for did in DRONE_IDS]
        
        rospy.Subscriber('/mission/dispatch_req', NavSatFix, self.request_cb)
        
        self.srv = rospy.Service('/fleet/get_status', Trigger, self.handle_get_status)
        
        rospy.loginfo(f"Fleet Manager Ready (Service Mode). Managing: {DRONE_IDS}")

    def handle_get_status(self, req):
        status_list = [agent.get_status_dict() for agent in self.agents]
        
        json_str = json.dumps(status_list)
        
        return TriggerResponse(success=True, message=json_str)

    def request_cb(self, msg):
        target_lat = msg.latitude
        target_lon = msg.longitude
        target_alt = msg.altitude if msg.altitude > 1 else 15.0
        
        rospy.loginfo(f"Dispatch Req: ({target_lat:.5f}, {target_lon:.5f})")
        
        available = [d for d in self.agents if d.is_available()]
        if not available:
            rospy.logwarn("No drones available!")
            return

        best = min(available, key=lambda d: d.distance_to(target_lat, target_lon))
        best.dispatch(target_lat, target_lon, target_alt)

if __name__ == '__main__':
    try:
        manager = FleetManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass