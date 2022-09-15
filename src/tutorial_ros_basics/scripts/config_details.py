#!/usr/bin/python3

import rospy

def main():

    rospy.init_node('config_details', anonymous=True)    
    param_config_my = rospy.get_param("details")

    first_name = param_config_my["name"]["first"]
    phone = param_config_my["contact"]["phone"]

    print(first_name)
    rospy.logwarn(first_name)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    