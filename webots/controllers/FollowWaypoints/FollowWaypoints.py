"""FollowWaypoints controller."""
"""Created by Mr.QB"""
import Crazyflie
if __name__ == "__main__":
    Crazyflie = Crazyflie() # Create robot

    Crazyflie.createTrajectory() # Create Trajectory for robot(circle with radius 0.05m)
    
    timestep = int(Crazyflie.robot.getBasicTimeStep())

    while Crazyflie.robot.step(timestep) != -1: # Run the script 
        Crazyflie.run()
