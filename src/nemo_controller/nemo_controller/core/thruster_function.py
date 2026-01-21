class thruster_functions:
    def __init__(self, thruster_map, max_vel_map, max_rpm):
        for i in range(0,len(thruster_map)):
            self.thruster[i] = thruster_map[0]
        self.max_surge = max_vel_map.get("surge")
        self.max_sway = max_vel_map.get("sway")
        self.max_heave = max_vel_map.get("heave")
        self.max_yaw = max_vel_map.get("yaw")
        self.max_rpm = max_rpm

    def surge(self,speed):
        self.thruster[0] = (speed/self.max_surge)*self.max_rpm
        self.thruster[1] = (speed/self.max_surge)*self.max_rpm
        self.thruster[2] = -1*(speed/self.max_surge)*self.max_rpm
        self.thruster[3] = -1*(speed/self.max_surge)*self.max_rpm
    
    def heave(self, speed):
        self.thruster[4] = -1 * (speed/self.max_surge) * self.max_rpm
        self.thruster[5] = (speed/self.max_surge) * self.max_rpm
        self.thruster[6] = (speed/self.max_surge) * self.max_rpm
        self.thruster[7] = -1 * (speed/self.max_surge) * self.max_rpm

    def sway(self,velocity):
        self.thruster[0] = -1 * (velocity/self.max_surge) * self.max_rpm
        self.thruster[1] = (velocity/self.max_surge) * self.max_rpm
        self.thruster[2] = -1 * (velocity/self.max_surge) * self.max_rpm
        self.thruster[3] = (velocity/self.max_surge) * self.max_rpm

    def yaw(self,velocity):
        self.thruster[0] = -1 * (velocity/self.max_surge) * self.max_rpm
        self.thruster[1] = (velocity/self.max_surge) * self.max_rpm
        self.thruster[2] = (velocity/self.max_surge) * self.max_rpm
        self.thruster[3] = -1 * (velocity/self.max_surge) * self.max_rpm