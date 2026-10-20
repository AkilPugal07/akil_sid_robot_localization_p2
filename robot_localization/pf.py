#!/usr/bin/env python3

"""This is the starter code for the robot localization project"""

from turtle import width
import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import ParticleCloud, Particle
from nav2_msgs.msg import Particle as Nav2Particle
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from rclpy.duration import Duration
import math
import time
import numpy as np
from occupancy_field import OccupancyField
from helper_functions import TFHelper
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler
from scipy.stats import norm
import random
import helper_functions as helper


class Particle(object):
    """Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
    Attributes:
        x: the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        theta: the yaw of the hypothesis relative to the map frame
        w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """Construct a new Particle
        x: the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        theta: the yaw of KeyboardInterruptthe hypothesis relative to the map frame
        w: the particle weight (the class does not ensure that particle weights are normalized
        """
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """A helper function to convert a particle to a geometry_msgs/Pose message"""
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )

    # TODO: define additional helper functions if needed


class ParticleFilter(Node):
    """The class that represents a Particle Filter ROS Node
    Attributes list:
        base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
        map_frame: the name of the map coordinate frame (should be "map" in most cases)
        odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
        scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
        n_particles: the number of particles in the filter
        d_thresh: the amount of linear movement before triggering a filter update
        a_thresh: the amount of angular movement before triggering a filter update
        pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
        particle_pub: a publisher for the particle cloud
        last_scan_timestamp: this is used to keep track of the clock when using bags
        scan_to_process: the scan that our run_loop should process next
        occupancy_field: this helper class allows you to query the map for distance to closest obstacle
        transform_helper: this helps with various transform operations (abstracting away the tf2 module)
        particle_cloud: a list of particles representing a probability distribution over robot poses
        current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                               The pose is expressed as a list [x,y,theta] (where theta is the yaw)
        thread: this thread runs your main loop
    """

    def __init__(self):
        super().__init__("pf")
        self.base_frame = "base_footprint"  # the frame of the robot base
        self.map_frame = "map"  # the name of the map coordinate frame
        self.odom_frame = "odom"  # the name of the odometry coordinate frame
        self.scan_topic = "scan"  # the topic where we will get laser scans from

        self.n_particles = 300  # the number of particles to use

        self.d_thresh = 0.2  # the amount of linear movement before performing an update
        self.a_thresh = (
            math.pi / 6
        )  # the amount of angular movement before performing an update

        # TODO: define additional constants if needed

        self.sensor_noise = 0.2
        self.random_reserve = 10
        self.noise_theta = math.pi / 16
        self.noise = 0.2
        self.beam_step = 5  # use every 5th lidar beam in the laser update
        # sensor noise for calculating SD on guassian dist. .2 as placeholder, could make ros param

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self.update_initial_pose, 10
        )

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(
            ParticleCloud, "particle_cloud", qos_profile_sensor_data
        )

        # laser_subscriber listens for data from the lidar
        self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        # your particle cloud will go here
        self.particle_cloud = []

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)

    def pub_latest_transform(self):
        """This function takes care of sending out the map to odom transform"""
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(
            seconds=0.1
        )
        self.transform_helper.send_last_map_to_odom_transform(
            self.map_frame, self.odom_frame, postdated_timestamp
        )

    def loop_wrapper(self):
        """This function takes care of calling the run_loop function repeatedly.
        We are using a separate thread to run the loop_wrapper to work around
        issues with single threaded executors in ROS2"""
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        """This is the main run_loop of our particle filter.  It checks to see if
        any scans are ready and to be processed and will call several helper
        functions to complete the processing.

        You do not need to modify this function, but it is helpful to understand it.
        """
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        new_pose, delta_t = self.transform_helper.get_matching_odom_pose(
            self.odom_frame, self.base_frame, msg.header.stamp
        )
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0):
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return

        r, theta = self.transform_helper.convert_scan_to_polar_in_robot_frame(
            msg, self.base_frame
        )

        # print out the first range and angle for debugging purposes
        # print("r[0]={0}, theta[0]={1}".format(r[0], theta[0]))
        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
            self.odom_pose
        )
        # print out the odometry pose for debugging purposes
        # print("x: {0}, y: {1}, yaw: {2}".format(*new_odom_xy_theta))

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particle_cloud:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            self.update_particles_with_odom()  # update based on odometry
            self.update_particles_with_laser(r, theta)  # update based on laser scan
            self.update_robot_pose()  # update robot's pose based on particles
            self.resample_particles()  # resample particles to focus on areas of high density
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return (
            math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0])
            > self.d_thresh
            or math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1])
            > self.d_thresh
            or math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2])
            > self.a_thresh
        )

    def update_robot_pose(self):
        """Update the estimate of the robot's pose given the updated particles.
        There are two logical methods for this:
            (1): compute the mean pose
            (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        # Computing robot pose using the mean method (using weighted averages)
        best = self.particle_cloud[0]
        weighted_x = 0
        weighted_y = 0
        weighted_theta = 0
        s = 0
        c = 0
        for p in self.particle_cloud:
            weighted_x += p.x * p.w
            weighted_y += p.y * p.w
            s += p.w * math.sin(p.theta)
            c += p.w * math.cos(p.theta)

        weighted_theta = math.atan2(s, c)

        # Creating type particle with the mean pose
        best = Particle(weighted_x, weighted_y, weighted_theta, 1.0)
        self.robot_pose = best.as_pose()
        # print out the robot pose for debugging purposes
        # print(f"pose: {weighted_x} {weighted_y} {weighted_theta}")

        # just to get started we will fix the robot's pose to always be at the origin
        if hasattr(self, "odom_pose"):
            self.transform_helper.fix_map_to_odom_transform(
                self.robot_pose, self.odom_pose
            )
        else:
            self.get_logger().warn(
                "Can't set map->odom transform since no odom data received"
            )

    def update_particles_with_odom(self):
        """Update the particles using the newly given odometry pose.
        The function computes the value delta which is a tuple (x,y,theta)
        that indicates the change in position and angle between the odometry
        when the particles were last updated and the current odometry.
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
            self.odom_pose
        )
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (
                new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                new_odom_xy_theta[2] - self.current_odom_xy_theta[2],
            )

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        # Wrong method since this was in the odom frame
        # for p in self.particle_cloud:
        #     p.x += delta[0]
        #     p.y += delta[1]
        #     p.theta += delta[2]

        # Convering odom frame to map frame
        old_theta = old_odom_xy_theta[2]
        local_dx = math.cos(old_theta) * delta[0] + math.sin(old_theta) * delta[1]
        local_dy = -math.sin(old_theta) * delta[0] + math.cos(old_theta) * delta[1]

        dtheta = self.transform_helper.angle_diff(new_odom_xy_theta[2], old_theta)
        # print out the odometry delta for debugging purposes
        # print(f"odom delta: {local_dx} {local_dy} {dtheta}")
        for p in self.particle_cloud:
            p.x += math.cos(p.theta) * local_dx - math.sin(p.theta) * local_dy
            p.y += math.sin(p.theta) * local_dx + math.cos(p.theta) * local_dy
            p.theta = self.transform_helper.angle_normalize(p.theta + dtheta)

    def resample_particles(self):
        """Resample the particles according to the new particle weights.
        The weights stored with each particle should define the probability that a particular
        particle is selected in the resampling step.  You may want to make use of the given helper
        function draw_random_sample in helper_functions.py.
        """
        # make sure the distribution is normalized
        self.normalize_particles()

        # Draw random samples based on particle weights
        weight_list = []
        for p in self.particle_cloud:
            weight_list.append(p.w)
        particle_cloud_prenoise = helper.draw_random_sample(
            self.particle_cloud, weight_list, self.n_particles - self.random_reserve
        )
        for p in particle_cloud_prenoise:
            p.x = p.x + random.uniform(self.noise * -1, self.noise)
            p.y = p.y + random.uniform(self.noise * -1, self.noise)
            p.theta = self.transform_helper.angle_normalize(
                p.theta + random.uniform(self.noise_theta * -1, self.noise_theta)
            )
            p.w = 1.0

        self.particle_cloud = []

        bounding_box = self.occupancy_field.get_obstacle_bounding_box()

        x_min = bounding_box[0][0]
        x_max = bounding_box[0][1]
        y_min = bounding_box[1][0]
        y_max = bounding_box[1][1]

        weight = 1.0

        # Add random particles to the particle cloud to maintain diversity
        for _ in range(self.random_reserve):
            temp_x = np.random.uniform(x_min, x_max)
            temp_y = np.random.uniform(y_min, y_max)
            temp_theta = np.random.uniform(0, 2 * math.pi)

            while not self.is_free(temp_x, temp_y):
                temp_x = np.random.uniform(x_min, x_max)
                temp_y = np.random.uniform(y_min, y_max)

            temp_particle = Particle(temp_x, temp_y, temp_theta, weight)
            self.particle_cloud.append(temp_particle)

        self.particle_cloud.extend(particle_cloud_prenoise)
        # print out the number of particles in the cloud to check if they stay constant
        # print(f"resampled: {len(self.particle_cloud)}")

        self.normalize_particles()

    def update_particles_with_laser(self, r, theta):
        """Updates the particle weights in response to the scan data
        r: the distance readings to obstacles
        theta: the angle relative to the robot frame for each corresponding reading
        """
        # Process laser scan data to create beams
        beams = []
        for i in range(0, len(r), self.beam_step):
            if math.isfinite(r[i]) and r[i] > 0:
                beams.append((r[i], theta[i]))

        if not beams:
            # print("no beams")
            return
        # print(f"beams: {len(beams)}")

        # Update particle weights based on the laser scan data
        for p in self.particle_cloud:
            if not self.is_free(p.x, p.y):
                p.w = 0.0
                continue
            dists = []
            for r_i, theta_i in beams:
                x_end = p.x + r_i * math.cos(p.theta + theta_i)
                y_end = p.y + r_i * math.sin(p.theta + theta_i)
                dists.append(
                    self.occupancy_field.get_closest_obstacle_distance(x_end, y_end)
                )
            new_w = np.nansum(norm.pdf(np.array(dists), loc=0, scale=self.sensor_noise))
            if math.isfinite(new_w):
                p.w = p.w * new_w
            else:
                p.w = 0.0
        # print out the maximum weight for debugging purposes
        # print(f"max w: {max(p.w for p in self.particle_cloud)}")

    def update_initial_pose(self, msg):
        """Callback function to handle re-initializing the particle filter based on a pose estimate.
        These pose estimates could be generated by another ROS Node or could come from the rviz GUI
        """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """Initialize the particle cloud.
        Arguments
        xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                  particle cloud around.  If this input is omitted, the odometry will be used
        """
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
                self.odom_pose
            )
        self.particle_cloud = []

        # completely disregards inputted pose, maybe generate random dist around given pose?
        bounding_box = self.occupancy_field.get_obstacle_bounding_box()

        # Get the bounding box coordinates
        x_min = bounding_box[0][0]
        x_max = bounding_box[0][1]
        y_min = bounding_box[1][0]
        y_max = bounding_box[1][1]

        weight = 1

        # Add random particles to the particle cloud to maintain diversity
        for _ in range(self.n_particles):
            temp_x = np.random.uniform(x_min, x_max)
            temp_y = np.random.uniform(y_min, y_max)
            temp_theta = np.random.uniform(0, 2 * math.pi)

            while not self.is_free(temp_x, temp_y):
                temp_x = np.random.uniform(x_min, x_max)
                temp_y = np.random.uniform(y_min, y_max)

            temp_particle = Particle(temp_x, temp_y, temp_theta, weight)
            self.particle_cloud.append(temp_particle)

        # print out the number of particles in the cloud to check if they stay constant
        # print(f"init: {len(self.particle_cloud)}")
        self.normalize_particles()
        self.update_robot_pose()

    def is_free(self, x, y):
        """Return True if (x, y) is a known free cell
        in the map (not unknown/outside or occupied)"""
        info = self.occupancy_field.map.info
        i = int((x - info.origin.position.x) / info.resolution)
        j = int((y - info.origin.position.y) / info.resolution)
        if i < 0 or j < 0 or i >= info.width or j >= info.height:
            return False
        return self.occupancy_field.map.data[i + j * info.width] == 0

    def normalize_particles(self):
        """Make sure the particle weights define a valid distribution (i.e. sum to 1.0)"""

        # Normalize the particle weights
        total_weight = sum(p.w for p in self.particle_cloud)
        if total_weight == 0:
            print("w all 0")
            for particle in self.particle_cloud:
                particle.w = 1 / self.n_particles
        else:
            for particle in self.particle_cloud:
                particle.w /= total_weight

        new_total = sum(p.w for p in self.particle_cloud)
        # print out the maximum weight to check they add up to 1.0
        # print(f"w sum: {new_total}")
        assert math.isclose(new_total, 1.0, abs_tol=1e-6)

    def publish_particles(self, timestamp):
        """
        Publish the current particle cloud to the particle_cloud topic
        """
        msg = ParticleCloud()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = timestamp
        for p in self.particle_cloud:
            msg.particles.append(Nav2Particle(pose=p.as_pose(), weight=p.w))
        self.particle_pub.publish(msg)

    def scan_received(self, msg):
        """Callback function to handle incoming laser scans"""
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop
        if self.scan_to_process is None:
            self.scan_to_process = msg


def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
