#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from likelihood_field import LikelihoodField

import numpy as np
from numpy.random import random_sample
import math
import copy

from random import randint, random



def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

#Function taken from class
def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob

def draw_random_sample(n, map, prob):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    #Did not use the function since this is essentially np.random.choice
    return

def get_inside_map(n, map):
    rand_idx = []
    for i in range(len(map)):
        if map[i] == 0:
            rand_idx.append(i)
    new_rand = np.random.choice(rand_idx, size = n)
    return new_rand

class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w
    
    def __str__(self):
        #taken from class
        theta = euler_from_quaternion([
            self.pose.orientation.x, 
            self.pose.orientation.y, 
            self.pose.orientation.z, 
            self.pose.orientation.w])[2]
        return ("Particle: [" + str(self.pose.position.x) + ", " + str(self.pose.position.y) + ", " + str(theta) + "]")



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        self.likelihood_field = LikelihoodField()

        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        while self.map.info.width == 0:
            pass
        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):

        self.map = data
    

    def initialize_particle_cloud(self):
        random_index = get_inside_map(self.num_particles, self.map.data)
        #print(self.map)
        for i in range(self.num_particles):
            p = Pose()
            p.position = Point()
            p.position.x = (random_index[i] % self.map.info.width) * self.map.info.resolution + self.map.info.origin.position.x
            #print("Particles initializing")
            # print("res : %f", self.map.info.resolution)
            # print("height: %f", self.map.info.height)
            p.position.y = (random_index[i] // self.map.info.height) * self.map.info.resolution + self.map.info.origin.position.y
            p.position.z = 0
            p.orientation = Quaternion()
            q = quaternion_from_euler(0, 0, random_sample() * 2 * (np.pi))
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            new_particle = Particle(p, 1.0)
            self.particle_cloud.append(new_particle)

        self.normalize_particles()

        self.publish_particle_cloud()

        print("particles initialized")


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        # gets the sum of all weights and divide each particle
        sum = 0
        for p in self.particle_cloud:
            sum += p.w
        for p in self.particle_cloud:
            p.w /= sum

        sum = 0
        for p in self.particle_cloud:
            sum += p.w
        print("SUM", sum)



    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):

        # resample particles via their weights.
        print("resampling particles")
        weight = []
        for p in self.particle_cloud:
            weight.append(p.w)
        resample_parts = np.random.choice(self.particle_cloud, self.num_particles, p = weight)
        for i in range(self.num_particles):
            #I first just assigned the resample parts but kept giving error and deepcopy fixed it
            self.particle_cloud[i] = copy.deepcopy(resample_parts[i])

    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if len(self.particle_cloud) != 0:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        # Take an average of all particles and set that as new pose
        sum_x = 0
        sum_y = 0
        sum_yaw = 0
        for p in self.particle_cloud:
            sum_x += p.pose.position.x
            sum_y += p.pose.position.y
            sum_yaw += get_yaw_from_pose(p.pose)
        sum_x /= self.num_particles
        sum_y /= self.num_particles
        sum_yaw /= self.num_particles
        self.robot_estimate.position.x = sum_x
        self.robot_estimate.position.y = sum_y
        q = quaternion_from_euler(0.0, 0.0, sum_yaw)
        self.robot_estimate.orientation.x = q[0]
        self.robot_estimate.orientation.y = q[1]
        self.robot_estimate.orientation.z = q[2]
        self.robot_estimate.orientation.w = q[3]


    def update_particle_weights_with_measurement_model(self, data):

        # TODO
        print("Updating particle weights")
        z_max = 4
        field = LikelihoodField()
        angles = [0, 45, 90, 135, 180, 225, 270, 315]
        #Initially used all 360 degrees but was too laggy
        for particle in self.particle_cloud:
            q = 1
            theta = get_yaw_from_pose(particle.pose)
            #for i, val in enumerate(data.ranges):
            for ang in angles:
                rad_i = ang * np.pi / 180
                val = data.ranges[ang]
                #print("in loop,, %d, %f", i, val)
                if val <= data.range_max and val != 0:
                    x_ztk = particle.pose.position.x + val*math.cos(theta + rad_i)
                    y_ztk = particle.pose.position.y + val*math.sin(theta + rad_i)
                    
                    dist = self.likelihood_field.get_closest_obstacle_distance(x_ztk, y_ztk)
                    #print("DIST")
                    #print(dist)
                    if math.isnan(dist):
                        #Since dist returns float nan for invalid entries we check
                        q = q * (0.8 * 0.00001 + 1)
                    else:
                        q = q * (0.8 * compute_prob_zero_centered_gaussian(dist, 0.1) +1)
            particle.w = q
            print("new particle weight is %f", q)
        

    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        diff_x = curr_x - old_x
        diff_y = curr_y - old_y
        diff_yaw = curr_yaw - old_yaw

        #added a random noise that has the standard deviation of 10 percent of original value
        diff_x += np.random.normal(0, abs(0.1*diff_x))
        diff_y += np.random.normal(0, abs(0.1*diff_y))
        diff_yaw += np.random.normal(0, abs(0.1*diff_yaw))

        for p in self.particle_cloud:
            p.pose.position.x += diff_x
            p.pose.position.y += diff_y
            q = quaternion_from_euler(0.0, 0.0, get_yaw_from_pose(p.pose) + diff_yaw)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]



if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









