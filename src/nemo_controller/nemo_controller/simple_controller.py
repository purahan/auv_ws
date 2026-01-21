import rclpy
import numpy as np
from rclpy.node import Node
import std_msgs
from std_msgs.msg import Float32,Float32MultiArray,Bool,Int8MultiArray
from nemo_interfaces.msg import RovCommands
from sensor_msgs.msg import Imu
from nemo_controller.core import thruster_function
import tf_transformations
import math

#joys] -> /input_cmd -> [mapping subscriber->[sim_thruster_controller]->(individual topic Publisher)]

class simpleThrusterController(Node):
    def __init__(self):
        super().__init__("sim_thruster_controller")
        
        #constants - to be parameterized later
        self.maxSurgeVelocity = 1.00
        self.maxSwayVelocity = 1.0
        self.heaveVelocity = 1.00
        self.maxYawVelocity = 0.25

        self.max_output_velocity = 5


        #Node Variables - similar to global volatile variable
        self.thruster_map = {
            "thruster1":0,
            "thruster2":0,
            "thruster3":0,
            "thruster4":0,
            "thruster5":0,
            "thruster6":0,
            "thruster7":0,
            "thruster8":0
        }

        self.thruster_values = [0,0,0,0,0,0,0,0]
        self.pitch = 0.0
        self.roll = 0.0
        self.pitch_rate = 0.0
        self.roll_rate = 0.0

        self.mapped_surge = 0.00
        self.mapped_heave = 0.00
        self.mapped_sway = 0.00
        self.mapped_yaw = 0.00

        self.max_thrust = 5.0

        self.roll_correction = 0.00
        self.pitch_correction = 0.00
        self.current_heave = 0.00
        self.current_depth = 0.00
        self.latched_heave = 0.00


        #status flags
        self.depth_lock = False
        self.arm_status = 0.0

        #publishers
        self.cmd_publisher = self.create_publisher(Int8MultiArray,"/nemo_auv/thruster_vals",10)

        #subscriber
        self.depth_status_subscriber = self.create_subscription(Bool,"/nemo_auv/lock_status",self.depth_lock_callback,10)
        self.arm_subscriber = self.create_subscription(Bool,'/nemo_auv/arm_status',self.arm_callback,10)
        self.cmd_subsriber = self.create_subscription(RovCommands,"/nemo_auv/input_cmd",self.cmdSubscriberCallback,10)
        self.depth_subscriber = self.create_subscription(Float32,'/nemo_auv/current_depth',self.depth_callback,10)


        #miscellaneous
        self.timer = self.create_timer(0.01,self.timer_callback)

        self.target_depth = 0.00
        self.target_heave = 0.00
        #creating a subscriber to subscribe to input cmd topic
        #creating a publisher to publish values to each of the 8 thruster topics
        


    def depth_callback(self,msg):
        self.current_depth = msg.data
        
    def depth_lock_callback(self, msg):
        if msg.data and not self.depth_lock:
            # Rising edge → latch values
            self.latched_heave = self.mapped_heave
            self.target_depth = self.current_depth

        self.depth_lock = msg.data
    
    def arm_callback(self, msg):
        self.arm_status = msg.data
        

    def imu_callback(self,msg):
        q = msg.orientation
        self.roll, self.pitch, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

        self.pitch_rate = msg.angular_velocity.y
        self.roll_rate = msg.angular_velocity.x

    def cmdSubscriberCallback(self,msg):
        
        forward_vel = msg.surge
        sway_vel = msg.sway
        heave_vel = msg.heave
        yaw_vel = msg.yaw

        self.mapped_surge = self.map(forward_vel,self.maxSurgeVelocity,self.max_output_velocity)
        self.mapped_sway = self.map(sway_vel,self.maxSwayVelocity,self.max_output_velocity)
        self.mapped_yaw = self.map(yaw_vel,self.maxYawVelocity,self.max_output_velocity)
        self.mapped_heave = self.map(heave_vel,self.heaveVelocity,self.max_output_velocity)

    def timer_callback(self):
        msg = Int8MultiArray()
        self.thrust_mixer(self.mapped_surge,self.mapped_sway,self.mapped_yaw,self.mapped_heave,0,0,self.depth_lock)
        msg_array = []
        for i in range(0,8):
            dict_str = f"thruster{i+1}"
            mapped_val = (self.thruster_map[dict_str] / 5.0) * 100
            msg_array.append(int(mapped_val))

        print(f"the published value is : [{msg_array}]")
        msg.data = msg_array
        self.cmd_publisher.publish(msg)
        print("message is published")
        

        
    def thrust_mixer(self, surge, sway, yaw, heave, roll, pitch,depth_lock):

        if self.arm_status:

            t = [0.00] * 8
            W_SURGE = 1.00
            W_SWAY  = 1.00
            W_YAW   = 0.50

            W_HEAVE = 1.00
            W_ROLL  = 0.75
            W_PITCH = 0.75

            MAX_THRUST = self.max_thrust

            pitch = pitch + self.surge_pitch_coupling(surge,0.15,0.25,-0.25)
            roll = roll + self.sway_roll_coupling(sway,0.075,0.15,-0.15)

            if depth_lock == True:
                heave_ = self.latched_heave
            else:
                heave_ = heave


            t[0] =  W_SURGE*surge - W_SWAY*sway - W_YAW*yaw
            t[1] =  W_SURGE*surge + W_SWAY*sway + W_YAW*yaw
            t[2] = -W_SURGE*surge - W_SWAY*sway + W_YAW*yaw
            t[3] = -W_SURGE*surge + W_SWAY*sway - W_YAW*yaw

            t[4] = W_HEAVE*heave_ + W_ROLL*roll + W_PITCH*pitch
            t[5] = W_HEAVE*heave_ - W_ROLL*roll + W_PITCH*pitch
            t[6] = W_HEAVE*heave_ + W_ROLL*roll - W_PITCH*pitch
            t[7] = W_HEAVE*heave_ - W_ROLL*roll - W_PITCH*pitch

            # ---------- Global Max-Norm Saturation ----------
            max_val = max(abs(v) for v in t)
            if max_val > MAX_THRUST:
                scale = MAX_THRUST / max_val
                t = [v * scale for v in t]

            for i in range(8):
                self.thruster_values[i] = t[i]
                self.thruster_map[f"thruster{i+1}"] = t[i]
        
        else:
            for i in range(8):
                self.thruster_values[i] = 0.0
                self.thruster_map[f"thruster{i+1}"] = t[i]



    def map(self,value,max_input_value,max_output_value):
        output_val = (value / max_input_value) * max_output_value

        return output_val
        
    def surge_pitch_coupling(self, surge_val, coupling_constant, sat_ul, sat_ll):
        """
        Passive surge-to-pitch coupling.
        Positive surge produces corrective pitch.

        surge_val           : forward velocity / command
        coupling_constant   : tuning gain (usually small, e.g. 0.05–0.2)
        sat_ul              : upper saturation limit
        sat_ll              : lower saturation limit
        """

        pitch_correction = coupling_constant * surge_val

        # Saturation
        if pitch_correction > sat_ul:
            pitch_correction = sat_ul
        elif pitch_correction < sat_ll:
            pitch_correction = sat_ll

        return pitch_correction

    def sway_roll_coupling(self, sway_val, coupling_constant, sat_ul, sat_ll):
        """
        Passive sway-to-roll coupling.
        Lateral motion produces corrective roll.

        sway_val            : lateral velocity / command
        coupling_constant   : tuning gain
        sat_ul              : upper saturation limit
        sat_ll              : lower saturation limit
        """

        roll_correction = coupling_constant * sway_val

        # Saturation
        if roll_correction > sat_ul:
            roll_correction = sat_ul
        elif roll_correction < sat_ll:
            roll_correction = sat_ll

        return roll_correction

    def clear_thrusters(self):
        for k in self.thruster_map:
            self.thruster_map[k] = 0.0
           
    def surge(self,velocity):
        self.thruster_map["thruster1"] = (velocity/self.maxSurgeVelocity) * self.max_output_velocity
        self.thruster_map["thruster2"] = (velocity/self.maxSurgeVelocity) * self.max_output_velocity
        self.thruster_map["thruster3"] = -(velocity/self.maxSurgeVelocity) * self.max_output_velocity
        self.thruster_map["thruster4"] = -(velocity/self.maxSurgeVelocity) * self.max_output_velocity
        
    
    def sway(self,velocity):
        self.thruster_map["thruster1"] = -(velocity/self.maxSwayVelocity) * self.max_output_velocity
        self.thruster_map["thruster2"] = (velocity/self.maxSwayVelocity) * self.max_output_velocity
        self.thruster_map["thruster3"] = -(velocity/self.maxSwayVelocity) * self.max_output_velocity
        self.thruster_map["thruster4"] = (velocity/self.maxSwayVelocity) * self.max_output_velocity

    
    def heave(self,velocity):
        self.thruster_map["thruster5"] = (velocity/self.heaveVelocity) * self.max_output_velocity
        self.thruster_map["thruster6"] = (velocity/self.heaveVelocity) * self.max_output_velocity
        self.thruster_map["thruster7"] = (velocity/self.heaveVelocity) * self.max_output_velocity
        self.thruster_map["thruster8"] = (velocity/self.heaveVelocity) * self.max_output_velocity

    
    def yaw(self,velocity):
        self.thruster_map["thruster1"] = -(velocity/self.maxYawVelocity) * self.max_output_velocity
        self.thruster_map["thruster2"] = (velocity/self.maxYawVelocity) * self.max_output_velocity
        self.thruster_map["thruster3"] = (velocity/self.maxYawVelocity) * self.max_output_velocity
        self.thruster_map["thruster4"] = -(velocity/self.maxYawVelocity) * self.max_output_velocity

def main(args=None):
    rclpy.init(args=args)
    node = simpleThrusterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()