import rospy

from hector_uav_msgs.srv import EnableMotors
from hector_uav_msgs.msg import Altimeter
from geometry_msgs.msg import Twist


cmd_pub= rospy.Publisher("cmd_vel", Twist ,queue_size=1)
def callback_takeoff(msg):
    z=msg.altitude
    u=0.1*(4-z)
    cmd_msg=Twist()
    cmd_msg.linear.z=u
    cmd_pub.publish(cmd_msg)





def main():
    rospy.init_node("test_control")
    # enable motor
    rospy.wait_for_service("/enable_motors")
    foo2call = rospy.ServiceProxy("/enable_motors",EnableMotors)
    if foo2call(True):
        print(" motor working")

    # get altitude data
    rospy.Subscriber("/altimeter",Altimeter, callback_takeoff)

    # compute control, u=k(z*-z)

    rospy.spin()
    






if __name__=="__main__":
    main()