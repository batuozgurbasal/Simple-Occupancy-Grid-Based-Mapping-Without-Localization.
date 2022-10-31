#!/usr/bin/env python
""" Simple occupancy-grid-based mapping without localization.

Subscribed topics:
/scan
/odom (new)

Published topics:
/map
/map_metadata

"""
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np


class Map(object):
    """
    The Map class stores an occupancy grid as a two dimensional
    numpy array.

    Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters.
        origin_x   --  Position of the grid cell (0,0) in
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.


    Note that x increases with increasing column number and y increases
    with increasing row number.
    """

    def __init__(self, origin_x=-5.0, origin_y=-5.0, resolution=0.01,
                 width=1000, height=1000):
        """ Construct an empty occupancy grid.

        Arguments: origin_x,
                   origin_y  -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells
                                in meters.
                   width,
                   height    -- The grid will have height rows and width
                                columns cells.  width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.

         The default arguments put (0,0) in the center of the grid.

        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.ones((height, width)) / 2

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """

        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message.
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height

        # Rotated maps are not supported... quaternion represents no
        # rotation.
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                                    Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).

        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        # print('-----------')

        # print (flat_grid)
        flat_grid = flat_grid.astype('int8')

        ##########

        grid_msg.data = list(np.round(flat_grid))

        return grid_msg

    def set_cell(self, x, y, val):
        """ Set the value of a cell in the grid.

        Arguments:
            x, y  - This is a point in the map coordinate frame.
            val   - This is the value that should be assigned to the
                    grid cell that contains (x,y).

        This would probably be a helpful method!  Feel free to throw out
        point that land outside of the grid.
        """
        pass


class Mapper(object):
    """
    The Mapper class creates a map from laser scan data.
    """

    def __init__(self):
        """ Start the mapper. """

        rospy.init_node('mapper')
        self._map = Map()

        # Setting the queue_size to 1 will prevent the subscriber from
        # buffering scan messages.  This is important because the
        # callback is likely to be too slow to keep up with the scan
        # messages. If we buffer those messages we will fall behind
        # and end up processing really old scans.  Better to just drop
        # old scans and always work with the most recent available.
        rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)

        # Latched publishers are used for slow changing topics like
        # maps.  Data will sit on the topic until someone reads it.
        self._map_pub = rospy.Publisher('map', OccupancyGrid, latch=True)
        self._map_data_pub = rospy.Publisher('map_metadata', MapMetaData, latch=True)

        rospy.spin()

    def odom_callback(self, msg):
        global roll, pitch, yaw
        global pos
        pos = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        print('yaw: ', yaw)
        print('pos: ', pos)
        print('pos.x: ', pos.x)

    def bayes(self, global_x, global_y, probability):
        if global_x >= 0 and global_x < self._map.width and global_y >= 0 and global_y < self._map.height:
            occured_probability = self._map.grid[global_x, global_y]

            # The probability of event B occurring, given event A has occurred * the probability of event A
            num = probability * occured_probability
            # The probability of event B
            denom = (probability * occured_probability) + (1 - probability) * (1 - occured_probability)
            # P(A|B) – the probability of event A occurring, given event B has occurred
            probability_final = num / denom

            return probability_final

    def get_indix(self, x, y):
        x = x - self._map.origin_x
        y = y - self._map.origin_y
        i = int(round(x / self._map.resolution))
        j = int(round(y / self._map.resolution))

        return i, j

    def scan_callback(self, scan):
        global global_x, global_y

        """ Update the map on every scan callback. """
        print('--------------------------------')
        print('the length of the range array is: ')
        print(len(scan.ranges))  # gives the range of the lidar sensor
        print('angle_min=', scan.angle_min)
        print('angle_max=', scan.angle_max)
        print('range_min=', scan.range_min)
        print('range_max=', scan.range_max)
        print('yaw_Scan: ', yaw)
        print('pos.x_scan: ', pos.x)
        print('pos.y_scan: ', pos.y)
        ###############updat the mape based on scan reading##########
        # you need to writr your code heer to update your map based on
        # sensor data

        # Fill some cells in the map just so we can see that something is
        # being published.
        # self._map.grid[0, 1] = .9
        # self._map.grid[0, 2] = .7
        # self._map.grid[1, 0] = .5
        # self._map.grid[2, 0] = 0.3

        # adjust the value of other grid and see what is happened on RVIZ:
        """
        for i  in range(self._map.width):
            self._map.grid[25, i] = 1
        for j in range(50):
            self._map.grid[j,j]=0.5
        """
        radius = 0.00000143  # radius of turtlebot3
        for i in range(len(scan.ranges)):  # For all the scan values from 0 to 360
            theta_s = math.radians(i)  ##converts the degree 'i' in scan.ranges[i] to radians.
            r = scan.ranges[i]  # for each laser value on the turtlebot3.
            if i < 10 or i > 350:  # for the values lower than 10 and higher than 350
                if not math.isinf(scan.ranges[i]):  # If the scan values are not infinity
                    if scan.ranges[i] > scan.range_min and scan.ranges[
                        i] < scan.range_max:  # If there is an obstacle detected
                        x_position = math.cos(theta_s) * (radius + r)
                        y_position = math.sin(theta_s) * (radius + r)
                        # theta_r = yaw
                        x_position_prime = (x_position * math.cos(yaw)) + (y_position * -math.sin(yaw))
                        y_position_prime = (x_position * math.sin(yaw)) + (y_position * math.cos(yaw))
                        # pos.x = Xr
                        # pos.y = Yr
                        x_position_double_prime = x_position_prime + pos.x
                        y_position_double_prime = y_position_prime + pos.y
                        (global_x, global_y) = self.get_indix(x_position_double_prime, y_position_double_prime)
                        self._map.grid[global_x, global_y] = self.bayes(global_x, global_y, 1)

                    # elif scan.ranges[i] < scan.range_min or scan.ranges[i] > scan.range_max:
                    #   (global_x, global_y ) = self.get_indix(x_position_double_prime,y_position_double_prime)
                    #  self._map.grid[global_x, global_y] = self.bayes(global_x,global_y,0.5)
                else:
                    (global_x, global_y) = self.get_indix(x_position_double_prime, y_position_double_prime)
                    self._map.grid[global_x, global_y] = self.bayes(global_x, global_y, 0)

        ############################################
        # Now that the map wass updated, so publish it!
        rospy.loginfo("Scan is processed, publishing updated map.")
        self.publish_map()

    def publish_map(self):
        """ Publish the map. """
        grid_msg = self._map.to_message()
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)


if __name__ == '__main__':
    try:
        m = Mapper()
    except rospy.ROSInterruptException:
        pass

"""
https://w3.cs.jmu.edu/spragunr/CS354_S14/labs/mapping/mapper.py
https://www.youtube.com/watch?v=K1ZFkR4YsRQ
"""
