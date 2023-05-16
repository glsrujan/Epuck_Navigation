"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# from controller import Node
# from controller import Robot
from controller import Supervisor
from controller import CameraRecognitionObject
# from scipy.stats.distributions import chi2
# import matplotlib.pyplot as plt
import numpy as np
import math
import sys


class EKF(object):
    def __init__(self):
        self.Sigma_n = np.zeros((2, 2))
        self.Sigma_m = None
        self.Sigma_x_t = np.zeros((3, 3))
        self.x_hat_t = None
        self.std_n_v = 0.001
        self.std_n_omega = (np.pi / 360)
        self.Sigma_x_t[0, 0], self.Sigma_x_t[1, 1], self.Sigma_x_t[2, 2] = 0.01, 0.01, np.pi / 90
        self.Sigma_n[0, 0] = self.std_n_v * self.std_n_v
        self.Sigma_n[1, 1] = self.std_n_omega * self.std_n_omega
        self.std_m = 0.05
        self.Sigma_m = [[self.std_m * self.std_m, 0], [0, self.std_m * self.std_m]]

    def EKFPropagate(self, u, dt):
        # TODO: Calculate the robot state estimation and variance for the next timestep

        self.x_hat_t[0] += (dt * (u[0]) * np.cos(self.x_hat_t[2]))
        self.x_hat_t[1] += (dt * (u[0]) * np.sin(self.x_hat_t[2]))
        self.x_hat_t[2] += (dt * (u[1]))

        phi = np.zeros((3, 3))
        phi[0, 0], phi[0, 1], phi[0, 2] = 1, 0, -dt * (u[0] * np.sin(self.x_hat_t[2]))
        phi[1, 0], phi[1, 1], phi[1, 2] = 0, 1, dt * (u[0] * np.cos(self.x_hat_t[2]))
        phi[2, 0], phi[2, 1], phi[2, 2] = 0, 0, 1

        g = np.zeros((3, 2))

        g[0, 0], g[0, 1] = -dt * np.cos(self.x_hat_t[2]), 0
        g[1, 0], g[1, 1] = -dt * np.sin(self.x_hat_t[2]), 0
        g[2, 0], g[2, 1] = 0, -dt

        self.Sigma_x_t = (phi @ self.Sigma_x_t @ phi.T) + (g @ self.Sigma_n @ g.T)
        return self.x_hat_t, self.Sigma_x_t

    def EKFRelPosUpdate(self, G_p_L, z):
        # TODO: Update the robot state estimation and variance based on the received measurement
        th = self.x_hat_t[2]
        C = np.zeros((2, 2))
        C[0, 0] = np.cos(th)
        C[0, 1] = -np.sin(th)
        C[1, 0] = np.sin(th)
        C[1, 1] = np.cos(th)

        G_p_L = np.reshape(G_p_L, (3, 1))
        G_p_Li = np.zeros((2, 1))
        G_p_Li[0, 0], G_p_Li[1, 0] = G_p_L[0, 0], G_p_L[1, 0]

        G_p_R = np.zeros((2, 1))
        G_p_R[0, 0] = self.x_hat_t[0]
        G_p_R[1, 0] = self.x_hat_t[1]

        z_est = (C.T @ (G_p_Li - G_p_R))
        z = np.reshape(z, (2, 1))
        r = z - z_est

        H = np.zeros((2, 3))
        H[0, 0] = -np.cos(th)
        H[0, 1] = -np.sin(th)
        H[0, 2] = (np.cos(th) * (G_p_Li[1, 0] - G_p_R[1, 0])) - (np.sin(th) * (G_p_Li[0, 0] - G_p_R[0, 0]))
        H[1, 0] = np.sin(th)
        H[1, 1] = -np.cos(th)
        H[1, 2] = -(np.sin(th) * (G_p_Li[1, 0] - G_p_R[1, 0])) - (np.cos(th) * (G_p_Li[0, 0] - G_p_R[0, 0]))

        S = (H @ self.Sigma_x_t @ H.T) + self.Sigma_m

        K = self.Sigma_x_t @ H.T @ np.linalg.inv(S)

        I = np.eye(3, dtype=float)
        self.Sigma_x_t = ((I - (K @ H)) @ self.Sigma_x_t @ np.transpose((I - (K @ H)))) + (
                K @ self.Sigma_m @ np.transpose(K))
        temp = np.reshape(self.x_hat_t, (3, 1))
        temp = temp + (K @ r)
        self.x_hat_t = np.reshape(temp, (1, 3))
        self.x_hat_t = self.x_hat_t[0]

        # return self.x_hat_t, self.Sigma_x_t


class EPuck(object):

    def __init__(self, wheelRadius, axleLength):
        self.position = None
        self.orientation = None
        self.wheelRadius = wheelRadius
        self.axleLength = axleLength
        self.G_goal = None
        self.R_goal = None
        self.G_T_R = None
        self.R_T_G = None
        self.bearing = None
        self.heading = 0.0
        self.Orientation_Flag = False
        self.vel = 0
        pass

    def set_goal(self, goal):
        self.G_goal = np.reshape(np.array([goal[0], goal[1], 0, 1]), (4, 1))

    def rotMat(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def update_pose(self, x_hat_t):
        G_R_R = self.rotMat(x_hat_t[2])
        G_pos_R = np.zeros((4, 1))
        G_pos_R[0, 0], G_pos_R[1, 0], G_pos_R[2, 0], G_pos_R[3, 0] = x_hat_t[0], x_hat_t[1], 0, 1
        self.position = G_pos_R
        self.orientation = G_R_R
        self.G_T_R = np.zeros((4, 4))
        self.G_T_R[0] = G_R_R[0, 0], G_R_R[0, 1], G_R_R[0, 2], G_pos_R[0, 0]
        self.G_T_R[1] = G_R_R[1, 0], G_R_R[1, 1], G_R_R[1, 2], G_pos_R[1, 0]
        self.G_T_R[2] = G_R_R[2, 0], G_R_R[2, 1], G_R_R[2, 2], G_pos_R[2, 0]
        self.G_T_R[3] = 0, 0, 0, 1
        self.R_T_G = np.linalg.inv(self.G_T_R)

    def get_dist(self, target_pos):
        dist = np.linalg.norm(self.position - target_pos)
        return dist

    def update_bearing(self):
        self.R_goal = self.R_T_G @ self.G_goal
        self.bearing = np.arctan2(self.R_goal[1, 0], self.R_goal[0, 0])

    def get_wheelSpeed(self, omega, lin_vel):
        wd = omega * self.axleLength * 0.5
        print("wheel velocities ",lin_vel - wd , lin_vel + wd)
        print("heading,omega,diff ",myPuck.heading,omega,omega-myPuck.heading)
        # if wd > 0.027:
        #     wd = 0.027
        return lin_vel - wd, lin_vel + wd

    def get_li_bearing(self):
        if self.bearing >= np.radians(90):
            li_bearing = np.radians(90)
        elif self.bearing <= np.radians(-90):
            li_bearing = np.radians(-90)
        else:
            li_bearing = self.bearing
        return li_bearing

    def epuck_controller(self, ranges):
        linear_vel = 0.06
        angular_vel = 0.5
        li_bearing = self.get_li_bearing()
        if not self.Orientation_Flag:
            if np.abs(self.bearing) > np.radians(1):
                self.heading = angular_vel
                self.vel = 0
                # print("turning", self.heading, self.vel)
                return
            else:
                self.Orientation_Flag = True
                print("Oriented")
        if np.mean(ranges[(255 - 24):(255 + 24)]) > 0.1:
            # print("if2")
            if ranges[255 - (int(np.degrees(li_bearing) / l_d))] > 0.2 and (ranges[255 + 128] > 0.2) and (
                    ranges[255 - 128] > 0.2):
                # print("if3")
                self.heading = 0.5*li_bearing
                self.vel = linear_vel
                print("bearing",li_bearing)
                # if np.abs(li_bearing)<np.radians(1):
                #     self.heading = li_bearing
                #     self.vel = linear_vel
                # else:
                #     self.heading = 0.5*li_bearing
                #     self.vel = linear_vel
            elif (ranges[255 + 128] > 0.13) and (ranges[255 - 128] > 0.13):
                # print("if4")
                self.heading = 0
                self.vel = linear_vel
            else:
                # print("else4")
                self.heading = lidar_out(ranges)
                self.vel = 0
        else:
            # print("else2")
            self.heading = lidar_out(ranges)
            self.vel = 0


def lidar_out(li_ranges):
    turn_vel = 0.5
    if np.abs(li_ranges[0] - li_ranges[511]) > 0.1:
        if li_ranges[0] >= li_ranges[511]:
            # print("turning Left")
            heading = turn_vel
        else:
            # print("turning Right")
            heading = -turn_vel
    else:
        if np.abs(np.mean(li_ranges[0:255]) - np.mean(li_ranges[255:511])) > 0.3:
            if np.mean(li_ranges[0:255]) >= np.mean(li_ranges[255:511]):
                # print("turning Left")
                heading = turn_vel
            else:
                # print("turning Right")
                heading = -turn_vel
        else:
            # print("turning Left")
            heading = turn_vel
    return heading


def get_temporarygoal(recObjs, recObjsNum):
    thetas = np.zeros(recObjsNum)
    if recObjsNum != 0:

        for i in range(0, recObjsNum):
            landmark = robot.getFromId(recObjs[i].get_id())
            thetas[i] = posOfImgToBearing(recObjs[i].get_position_on_image()[0])
            G_p_L = landmark.getPosition()
            return G_p_L ,thetas
            # print(G_p_L)
            pass
    else:
        # print("No objects found")
        G_p_L = GOAL
        pass

    return G_p_L , thetas

def posOfImgToBearing(x):
    # TODO: Calculate the bearing measurement based on landmark's position on image
    fov = 0.84  # the field of view of the camera (rad)
    w = 640  # the width of camera (pixels)
    d = (0.5 * w) / np.tan(0.5 * fov)
    R_th_Li = np.arctan2((0.5 * w) - x, d)
    return R_th_Li


if __name__ == "__main__":

    # Create EPuck object
    global GOAL
    # goal_x = float(input("Enter X Co-ordinate of the Goal: "))
    # goal_y = float(input("Enter X Co-ordinate of the Goal: "))
    goal_x = 2.86
    goal_y = -0.12
    thetas = np.zeros(0)

    GOAL = [goal_x, goal_y]
    goal_waypoint = GOAL
    wheelRadius = 0.0205
    axleLength = 0.0568  # Data from Webots website seems wrong. The real axle length should be about 56-57mm
    # TODO: Update Frequency has to be looked into
    updateFreq = 100  # update every 200 timesteps
    counter = 0
    myPuck = EPuck(wheelRadius, axleLength)
    myPuck_EKF = EKF()

    # Set Destination Goal
    myPuck.set_goal(goal_waypoint)
    # create the Robot instance.
    robot = Supervisor()
    camera = robot.getDevice('camera')
    camera.enable(1)
    camera.recognitionEnable(1)
    camera.enableRecognitionSegmentation()
    lidar = robot.getDevice('lidar')
    lidar.enable(1)
    lidar.enablePointCloud()

    if camera.hasRecognition():
        camera.recognitionEnable(1)
        camera.enableRecognitionSegmentation()
    else:
        print("Your camera does not have recognition")

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))

    # Initialise Variables
    dt = timestep / 1000.0
    # vel = 0
    l_d = 180 / 512

    # Initialise Robot
    robotNode = robot.getFromDef("e-puck")
    G_p_R = robotNode.getPosition()
    G_ori_R = robotNode.getOrientation()
    G_ori_R = np.reshape(G_ori_R, (3, 3))
    theta = np.arctan2(G_ori_R[1, 0], G_ori_R[0, 0])
    myPuck_EKF.x_hat_t = np.array([G_p_R[0], G_p_R[1], theta])

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:

        left_v, right_v = myPuck.get_wheelSpeed(myPuck.heading + np.random.normal(0, myPuck_EKF.std_n_omega),
                                                myPuck.vel + np.random.normal(0, myPuck_EKF.std_n_v))

        # left_v, right_v = myPuck.get_wheelSpeed(myPuck.heading,
        #                                         myPuck.vel)
        # print(left_v,right_v)
        leftMotor.setVelocity(left_v / myPuck.wheelRadius)
        rightMotor.setVelocity(right_v / myPuck.wheelRadius)
        u = np.array([myPuck.vel, myPuck.heading])
        myPuck_EKF.EKFPropagate(u, dt)
        # print(myPuck_EKF.x_hat_t)

        recObjs = camera.getRecognitionObjects()
        recObjsNum = camera.getRecognitionNumberOfObjects()
        z_pos = np.zeros((recObjsNum, 2))  # relative position measurements

        if counter % updateFreq == 0:
            counter = 0
            thetas = np.zeros(recObjsNum)
            for i in range(0, recObjsNum):

                landmark = robot.getFromId(recObjs[i].get_id())
                G_p_Land = landmark.getPosition()
                thetas[i] = posOfImgToBearing(recObjs[i].get_position_on_image()[0])
                rel_lm_trans = landmark.getPose(robotNode)

                std_m = 0.05
                Sigma_m = [[std_m * std_m, 0], [0, std_m * std_m]]
                z_pos[i] = [rel_lm_trans[3] + np.random.normal(0, std_m), rel_lm_trans[7] + np.random.normal(0, std_m)]
                myPuck_EKF.EKFRelPosUpdate(G_p_Land, z_pos[i])
        counter = counter + 1

        myPuck.update_pose(myPuck_EKF.x_hat_t)
        myPuck.update_bearing()

        ranges = np.array(lidar.getRangeImage())
        ranges[ranges == math.inf] = 3

        # GPL, th = get_temporarygoal(recObjs, recObjsNum)
        if thetas.size != 0:
            myPuck.bearing = np.max(thetas)
        # myPuck.set_goal()

        # Turn Towards the target
        myPuck.epuck_controller(ranges)
        target_pos = np.reshape(np.array([GOAL[0], GOAL[1], 0, 1]), (4, 1))
        delta_dist = myPuck.get_dist(target_pos)
        # print(delta_dist)
        if delta_dist < 0.1:
            print("Congratulation!! e-Puck reached the Goal")
            quit()
    pass
# Enter here exit cleanup code.
