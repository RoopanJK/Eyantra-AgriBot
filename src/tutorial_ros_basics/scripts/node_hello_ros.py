#!/usr/bin/python3

import rospy

def main():

    rospy.init_node('node_hello_ros',anonymous=True) #To make it as a ROS node

    rospy.loginfo("Hello World") #Print into console

    rospy.spin() #Keep the node running until terminated by user


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

