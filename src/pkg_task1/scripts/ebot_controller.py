#!/usr/bin/python3


from math import sqrt, pow, pi ,cos
from numpy.lib.function_base import angle
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

waypoint_x = [-1.1, -1.1, 1.05, 0.8, 2.7, 2.6]
waypoint_y = [-1, 8.5, 8.5, -1.4, -1.1, 8.2]

angle_z = [3.06, 1.54, 0, 4.75, 0, 1.59]



class controller:

    def __init__(self) -> None:
        self.z_ang = 0
        self.regions = 0
        self.px = 0
        self.py = 0
        self.lin_tol = 0.05
        self.ang_tol = 0.05
        self.d = 0.65
        rospy.init_node("ebot_controller")
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/ebot/laser/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def laser_callback(self, msg):
        self.regions = {
            'right': min(min(msg.ranges[0:143]), 8),
            'fright': min(min(msg.ranges[144:287]), 8),
            'front':  min(min(msg.ranges[288:431]), 8),
            'fleft':  min(min(msg.ranges[432:575]), 8),
            'left':  min(min(msg.ranges[576:713]), 8),
        }

    def odom_callback(self, data):
        self.px = data.pose.pose.position.x
        self.py = data.pose.pose.position.y
        x = data.pose.pose.orientation.x
        y = data.pose.pose.orientation.y
        z = data.pose.pose.orientation.z
        w = data.pose.pose.orientation.w
        self.z_ang = euler_from_quaternion([x, y, z, w])[2]

    #Normalization for rotate
    def normailize(self, ang_z):
        if(ang_z < 0):
            ang = pi - ang_z
        else:
            ang = ang_z
        print(ang)
        return ang
        
    #Rotate 
    def rotate(self, i,direction):
        rotate_msg = Twist()
        while((abs(self.normailize(self.z_ang) - i)) > self.ang_tol):
            rotate_msg.angular.z = 0.5 * direction
            self.pub.publish(rotate_msg)
        rotate_msg.angular.z = 0
        self.pub.publish(rotate_msg)

    #Euc distance
    def euc(self, x, y):
        ed = sqrt(pow(( x - self.px), 2) + pow(( y - self.py), 2))
        return ed

    #Move to goal algorithm
    def movetogoal(self, gx, gy,tolerance=0.5):
        movetogoal_msg = Twist()
        while(self.euc(gx, gy) > tolerance):
            movetogoal_msg.linear.x = 0.5
            self.pub.publish(movetogoal_msg)
        movetogoal_msg.linear.x = 0
        self.pub.publish(movetogoal_msg)

if __name__ == '__main__':
    try:
        go = controller()
        print("In main")
        rospy.sleep(3)
        go.rotate(angle_z[0],1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[0], waypoint_y[0])
        rospy.sleep(0.5)
        go.rotate(angle_z[1],-1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[1], waypoint_y[1])
        rospy.sleep(0.5)
        go.rotate(angle_z[2],-1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[2], waypoint_y[2])
        rospy.sleep(0.5)
        go.rotate(angle_z[3], -1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[3], waypoint_y[3])
        rospy.sleep(0.5)
        go.rotate(angle_z[4], 1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[4], waypoint_y[4])
        rospy.sleep(0.5)
        go.rotate(angle_z[5], 1)
        rospy.sleep(0.5)
        go.movetogoal(waypoint_x[5], waypoint_y[5])

    except rospy.ROSInterruptException:
        pass
