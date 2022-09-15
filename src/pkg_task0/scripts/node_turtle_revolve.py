#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class turtlesim:                

    #Initialization

    def __init__(self):
        rospy.init_node('node_turtle_revolve', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.poseCallback)
        self.pose = Pose()

    #Subscriber Callback 

    def poseCallback(self, data):
        self.pose.theta = round(data.theta, 1)
        rospy.loginfo("Theta = %f\n", self.pose.theta)

    def move_circle(self, direction):
        """
        @brief publishes velocity to make turtle move in circle
        @param direction -1 to move backward +1 to move forward
        """
        velocity_msg = Twist()
        velocity_msg.linear.x = 2
        velocity_msg.angular.z = abs(2/2)*direction
        rate = rospy.Rate(10)
        count = 0
        angle = 3.10
        while not rospy.is_shutdown():
            self.velocity_publisher.publish(velocity_msg)
            rate.sleep()
            if self.pose.theta == angle:
                angle = -0.0
                count = count + 1
                continue
            elif count == 2:
                break
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
        self.velocity_publisher.publish(velocity_msg)
        rospy.loginfo("Complete")
        rate.sleep()
    
if __name__ == "__main__":
    try:
        x = turtlesim()
        x.move_circle(-1)
        x.move_circle(1)
    
    except rospy.ROSInterruptException:
        pass
