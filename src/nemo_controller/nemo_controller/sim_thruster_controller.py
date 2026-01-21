import rclpy
import numpy as np
from rclpy.node import Node
import std_msgs
from std_msgs.msg import Float64
from nemo_interfaces.msg import RovCommands
from sensor_msgs.msg import Imu
from nemo_controller.core import thruster_function
import tf_transformations
import math

#joys] -> /input_cmd -> [mapping subscriber->[sim_thruster_controller]->(individual topic Publisher)]

class simThrusterController(Node):
    def __init__(self):
        super().__init__("sim_thruster_controller")
        

        self.maxSurgeVelocity = 1.00
        self.maxSwayVelocity = 1.0
        self.heaveVelocity = 1.00
        self.maxYawVelocity = 0.25

        self.max_output_velocity = 5

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

        self.pitch_i = 0.0

        self.mapped_surge = 0.00
        self.mapped_heave = 0.00
        self.mapped_sway = 0.00
        self.mapped_yaw = 0.00

        self.roll_correction = 0.00
        self.pitch_correction = 0.00
        
        #allocation function
        self.allocatorMatrix = np.array([
            [0.707, 0.707, -0.707, -0.707, 0, 0, 0, 0],
            [-0.707, 0.707, -0.707, 0.707, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [-0.293, +0.293, +0.293, -0.293, 0, 0, 0, 0],
        ])

        #pseudo Inverse
        self.Binv = np.linalg.pinv(self.allocatorMatrix)

        #gain matrix 
        self.gain_matrix = np.array([
            [0.5, 0, 0, 0, 0, 0],
            [0, 0.5, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0.5],
        ])
        #creating a subscriber to subscribe to input cmd topic
        self.cmd_subsriber = self.create_subscription(RovCommands,"/input_cmd",self.cmdSubscriberCallback,10)
        self.publisher = []
        #creating a publisher to publish values to each of the 8 thruster topics
        for i in range(8):
            topic_str = f"/nemo_auv/thruster{i+1}/cmd"
            self.publisher.append(
                self.create_publisher(Float64, topic_str, 10)
            )

        self.imu_subscriber = self.create_subscription(Imu,'/IMU/VectorNav1',self.imu_callback,100)
        self.timer = self.create_timer(0.01,self.timer_callback)




    def imu_callback(self,msg):
        q = msg.orientation
        self.roll, self.pitch, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

        self.pitch_rate = msg.angular_velocity.y
        self.roll_rate = msg.angular_velocity.x

        self.clear_thrusters()

        self.roll_correction = self.roll_controller(self.roll,self.roll_rate,0.25,-0.25)
        self.pitch_correction = self.pitch_controller(self.pitch,self.pitch_rate,0.25,-0.25)

        

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

        self.horizontal_thruster_mixer(self.mapped_surge,self.mapped_sway,self.mapped_yaw)
        self.vertical_thruster_mixer(self.mapped_heave,self.roll_correction,self.pitch_correction)

        for i in range(0,8):
            msg=Float64()
            thrusterid = f"thruster{i+1}"
            msg.data = (self.thruster_map.get(thrusterid))
            self.publisher[i].publish(msg)
            print(f"published the value - {self.thruster_map.get(thrusterid)} \n")
        
    def horizontal_thruster_mixer(self, surge, sway, yaw):

        t = [0.0]*4

        W_SURGE = 1
        W_SWAY = 1
        W_YAW = 0.75

        MAX_THRUST = 5

        # Weighted mix
        t[0] =  W_SURGE*surge - W_SWAY*sway - W_YAW*yaw
        t[1] =  W_SURGE*surge + W_SWAY*sway + W_YAW*yaw
        t[2] = -W_SURGE*surge - W_SWAY*sway + W_YAW*yaw
        t[3] = -W_SURGE*surge + W_SWAY*sway - W_YAW*yaw

        # Max-norm saturation
        max_val = max(abs(v) for v in t)
        if max_val > MAX_THRUST:
            scale = MAX_THRUST / max_val
            t = [v * scale for v in t]

        # Store
        for i in range(4):
            self.thruster_values[i] = t[i]
            self.thruster_map[f"thruster{i+1}"] = t[i]

    def vertical_thruster_mixer(self, heave, roll, pitch):

        t = [0.0]*4

        W_HEAVE = 1
        W_ROLL = 0.75
        W_PITCH = 0.75

        MAX_THRUST = 5

        t[0] = W_HEAVE*heave + W_ROLL*roll + W_PITCH*pitch
        t[1] = W_HEAVE*heave - W_ROLL*roll + W_PITCH*pitch
        t[2] = W_HEAVE*heave + W_ROLL*roll - W_PITCH*pitch
        t[3] = W_HEAVE*heave - W_ROLL*roll - W_PITCH*pitch

        max_val = max(abs(v) for v in t)
        if max_val > MAX_THRUST:
            scale = MAX_THRUST / max_val
            t = [v * scale for v in t]

        for i in range(4):
            self.thruster_values[i+4] = t[i]
            self.thruster_map[f"thruster{i+5}"] = t[i]


    def map(self,value,max_input_value,max_output_value):
        output_val = (value / max_input_value) * max_output_value

        return output_val
        
    def roll_controller(self, roll_angle, roll_rate, max_corr, min_corr):
        """
        roll_angle : rad
        roll_rate  : rad/s
        """

        # ---------- Tunable gains ----------
        Kp_angle = 2.0     # outer loop
        kd_angle = 0.5

        Kp_rate  = 1     # inner loop
        Kd_rate  = 0.02
        ki_rate = 0.25

        # ---------- Stage 1: Angle loop ----------
        angle_error = -roll_angle
        desired_rate = Kp_angle * angle_error + (kd_angle * angle_error) 

        # ---------- Stage 2: Rate loop ----------
        rate_error = desired_rate - roll_rate
        correction = (Kp_rate * rate_error) + (Kd_rate * roll_rate) + (ki_rate * rate_error) 

        # ---------- Saturation ----------
        correction = max(min(correction, max_corr), min_corr)

        return correction

    def pitch_controller(self, pitch_angle, pitch_rate, max_corr, min_corr, dt=0.01):

        Kp_angle = 3
        Ki_angle = 0.8     # << this fixes steady-state error
        

        Kp_rate = 1.0
        Kd_rate = 0.05

        # --- Outer loop ---
        angle_error = -pitch_angle
        self.pitch_i += angle_error * dt
        self.pitch_i = np.clip(self.pitch_i, -0.3, 0.3)

        desired_rate = Kp_angle * angle_error + Ki_angle * self.pitch_i

        # --- Inner loop ---
        rate_error = desired_rate - pitch_rate
        correction = Kp_rate * rate_error - Kd_rate * pitch_rate

        if correction >= max_corr:
            return max_corr
        elif correction <= min_corr:
            return min_corr
        else:
            return correction


    def depth_controller(self,depth,rate):

        ...
    
    def pid():
        ...
    def clear_thrusters(self):
        for k in self.thruster_map:
            self.thruster_map[k] = 0.0
        

    #gets the velocity vector and returns the Thruster Output
    def thrusterAllocatorfunc(self,velocityVector):
        v = np.array(velocityVector)

        #Tout is the output thruster Vector
        Tout = self.Binv @ (self.gain_matrix @ v)
        return Tout

    
    
    
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
    node = simThrusterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()