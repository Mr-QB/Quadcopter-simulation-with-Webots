from controller import Robot, Motor, GPS, LED, InertialUnit, Gyro, Compass
from simple_pid import PID
import numpy as np

class Crazyflie:
    def __init__(self):
        self.robot = Robot()
        # Setup MOTOR for Cazyflie robot
        self.motor_names = ['m1_motor','m2_motor','m3_motor','m4_motor'] 
        self.motors = []
        for motor_names in self.motor_names:
            motor = self.robot.getDevice(motor_names)
            motor.setPosition(float('inf'))
            self.motors.append(motor)
        # Setup GPS for Cazyflie robot
        self.gps = GPS("gps")
        self.gps.enable(8)
        # Setup IMU for Cazyflie robot
        self.imu = InertialUnit("inertial unit")
        self.imu.enable(8) 
        # Setup GYRO for Cazyflie robot
        self.gyro = Gyro("gyro")
        self.gyro.enable(8)
        # Setup function PID (parameters Kp,Ki,Kd not fully optimized yet)
        self.throttlePID = PID(19, 5.3 ,15, setpoint=0.1)
        self.pitchPID = PID(4.97, 0.013, 4.57, setpoint=0.1)
        self.rollPID = PID(4.92, 0.013, 4.57, setpoint=0.1)
        self.yawPID = PID(3.2, 0.013, 2.1, 0.7)
        # create current pose(Start point) 
        self.current_pose = [0,0,0]

        self.pointer = 0
        self.target = [0,0,0]
        self.waypoint = self.createTrajectory()
        return


    def getMotorAll(self): # Get and enable 4 motor of Crazyflie robot
        m1_motor = self.robot.getDevice('m1_motor')
        m2_motor = self.robot.getDevice('m1_motor')
        m3_motor = self.robot.getDevice('m1_motor')
        m4_motor = self.robot.getDevice('m1_motor')
        return [m1_motor, m2_motor, m3_motor, m4_motor]


    def motorsSpeed(self,velocity): # Set/update velocity for 4 motor of Crazyflie robot
        motor = self.getMotorAll()
        for i in range(0,4):
            motor[i].setVelocity(velocity[i])
        return
    def findWaypoint(self):
        # waypoint = [targetX, targetY, target_altitude] calculated according to the trajectory generation function
        return [[2.0, 1.0, 2.0]]
    def updateTheseCurren(self):
        # Orientation
        self.roll = self.imu.getRollPitchYaw()[0]
        self.pitch = self.imu.getRollPitchYaw()[1]
        self.yaw = self.imu.getRollPitchYaw()[2]
        
        # Acceleration  Velocities
        self.roll_acceleration = self.gyro.getValues()[0]
        self.pitch_acceleration = self.gyro.getValues()[1]

        # Position
        self.xGPS = self.gps.getValues()[0]
        self.yGPS = self.gps.getValues()[1]
        self.zGPS = self.gps.getValues()[2]
        
        return

    def clamp(self,value, value_min, value_max):
        return min(max(value, value_min), value_max)

    def error(self,des,now_):
        return abs(des-now_)

    def find_angle(self,target_position,current_pose):
        angle_left = np.arctan2((target_position[0]-0), (target_position[1]-0))
        return [angle_left,angle_left/abs(angle_left)]
        

    def createTrajectory(self):

        return [[0.06,0.05,0.1],[0.06,0.06,0.1]]
    def getTarget(self):
        self.updateTheseCurren()
        target_yaw = self.find_angle(self.target,self.current_pose)
        print(self.yaw,target_yaw[0])
        if(self.error(target_yaw[0],self.yaw)<0.01):
            print("true")
        if(self.error(self.target[0],self.xGPS)<0.01 and self.error(self.target[1],self.yGPS)<0.01 and self.error(self.target[2],self.zGPS)<0.01 and self.error(target_yaw[0],self.yaw)<0.01):
            print("pose: ",self.pointer)
            self.current_pose = self.waypoint[self.pointer-1]
            self.pointer += 1
            if self.pointer >= len(self.waypoint):
                self.pointer = 0
        self.target = self.waypoint[self.pointer]
        return 

    def findSpeedMotor(self):# target is a waypoint
        # Update target for PID function
        Crazyflie.getTarget()
        self.rollPID.setpoint = self.target[1]
        self.pitchPID.setpoint = self.target[0]
        self.throttlePID.setpoint = self.target[2]
        target_yaw = self.find_angle(self.target,self.current_pose)
        self.yawPID.setpoint = target_yaw[0]
        
        # Get these curren
        self.updateTheseCurren()

        vertical_input = round(self.throttlePID(round(self.zGPS,3)),2)
        roll_input = 30*self.clamp(self.roll, -1, 1) + self.roll_acceleration  + self.rollPID(self.yGPS)
        pitch_input =  -15*self.clamp(self.pitch, -1, 1) - self.pitch_acceleration + self.pitchPID(self.xGPS)
        yaw_input = -self.yawPID(abs(self.yaw))


        # Calc velocity for 4 motor of robot 
        m1_input = round(54 + vertical_input + roll_input - pitch_input - yaw_input,3)
        m2_input = round(54 + vertical_input + roll_input + pitch_input + yaw_input,3)
        m3_input = round(54 + vertical_input - roll_input + pitch_input - yaw_input,3)
        m4_input = round(54 + vertical_input - roll_input - pitch_input + yaw_input,3)

        # Check min/max of velocity
        m1_input = self.clamp(m1_input,-600,600)
        m2_input = self.clamp(m2_input,-600,600)
        m3_input = self.clamp(m3_input,-600,600)
        m4_input = self.clamp(m4_input,-600,600)

        # Update Current pose
        return [-m1_input,m2_input,-m3_input,m4_input]

    def motorsSpeed(self,velocity):
        for i in range(0,4):
            self.motors[i].setVelocity(velocity[i])
        return

    def run(self):
        motor_speed = self.findSpeedMotor()
        self.motorsSpeed(motor_speed)


        
        return


if __name__ == "__main__":
    Crazyflie = Crazyflie() # Create robot

    Crazyflie.createTrajectory() # Create Trajectory for robot
    
    timestep = int(Crazyflie.robot.getBasicTimeStep())

    while Crazyflie.robot.step(timestep) != -1: # Run the script 
        Crazyflie.run()
