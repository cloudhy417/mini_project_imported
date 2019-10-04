#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from geometry_msgs.msg import Pose2D
import qi
import argparse
import sys
import math
import almath




class controller():
    
    def __init__(self):
        
        parser = argparse.ArgumentParser()
        parser.add_argument("--ip", type=str, default="192.168.11.28",
                            help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
        parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
        self.session = qi.Session()
        args = parser.parse_args()
        try:
            self.session.connect("tcp://" + args.ip + ":" + str(args.port))
        except RuntimeError:
            print("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) + ".\n"
                  "Please check your script arguments. Run with -h option for help.")
            sys.exit(1)


    def control(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %f %f %f",
                  data.x, data.y, data.theta)
        """
        Move To: Small example to make Nao Move To an Objective.
        """
        # Get the services ALMotion & ALRobotPosture.

        motion_service = self.session.service("ALMotion")
        posture_service = self.session.service("ALRobotPosture")

        # Wake up robot
        motion_service.wakeUp()

        # Send robot to Stand Init
        posture_service.goToPosture("StandInit", 0.5)

        #####################
        # Enable arms control by move algorithm
        #####################
        motion_service.setMoveArmsEnabled(True, True)
        #~ motion_service.setMoveArmsEnabled(False, False)

        #####################
        # FOOT CONTACT PROTECTION
        #####################
        #~ motion_service.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION",False]])
        motion_service.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

        #####################
        # get robot position before move
        #####################
        initRobotPosition = almath.Pose2D(motion_service.getRobotPosition(False))

        X = data.x
        Y = data.y
        Theta = data.theta

        motion_service.moveTo(X, Y, Theta, _async=True)
        # wait is useful because with _async moveTo is not blocking function
        motion_service.waitUntilMoveIsFinished()

        #####################
        # get robot position after move
        #####################
        endRobotPosition = almath.Pose2D(motion_service.getRobotPosition(False))

        #####################
        # compute and print the robot motion
        #####################
        robotMove = almath.pose2DInverse(initRobotPosition)*endRobotPosition
        # return an angle between ]-PI, PI]
        robotMove.theta = almath.modulo2PI(robotMove.theta)
        print "Robot Move:", robotMove

        # Go to rest position
def control(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %f %f %f",
                  data.x, data.y, data.theta)
def my_pepper_controller():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('my_pepper_controller', anonymous=True)
    mycontroller = controller()
    rospy.Subscriber("pepper_controller_command", Pose2D, control)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':

    my_pepper_controller()

# ros2 topic pub -1 /pepper_controller_command geometry_msgs/Pose2D "x: 0.0
# y: 0.0
# theta: 0.0"
