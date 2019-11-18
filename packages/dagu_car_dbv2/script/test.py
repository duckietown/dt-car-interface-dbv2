#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdDBV2Stamped, Twist2DStamped, BoolStamped
from duckietown_msgs.srv import SetValueRequest, SetValueResponse, SetValue
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from numpy import *
import math                                                                     #for the calculations of the wheels command (sqrt, )
import yaml
import time
import os.path
from duckietown_utils import get_duckiefleet_root

# Inverse Kinematics Node
# Author: Robert Katzschmann, Shih-Yuan Liu

class InverseKinematicsNode(object):
    def __init__(self):
        # Get node name and vehicle name
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]

        # Set parameters using yaml file
        self.readParamFromFile()

        # Set local variable by reading parameters (ev. use negative values, because it is easier to debug then)
        self.gain_dc = self.setup_parameter("~gain_dc", 0.6)                    #changed "gain" to "gain_dc", to identify uniquely for the dc-motor, RFMH_2019_02_25
        self.trim_dc = self.setup_parameter("~trim_dc", 0.0)                    #changed "trim" to "trim_dc", to identify uniquely for the dc-motor, RFMH_2019_02_25
         # self.gain_servo = self.setup_parameter("~gain_servo", 0.1)              #changed "gain" to "gain_servo", to identify uniquely for the servo-motor, RFMH_2019_02_25
         # self.trim_servo = self.setup_parameter("~trim_servo", 0.0)              #changed "trim" to "trim_servo", to identify uniquely for the servo-motor, RFMH_2019_02_25
        self.baseline = self.setup_parameter("~baseline", 0.092)                #adjusted car width , RFMH_2019_02_25
        self.radius = self.setup_parameter("~radius", 0.019)                    #adjusted wheel radius to 0.019m , RFMH_2019_02_25
        self.k = self.setup_parameter("~k", 27.0)                               #maybe needs adjustment, nothing changed yet , RFMH_2019_02_25
         # self.axis_distance = self.setup_parameter("~axis_distance", 0.105)      #introduce the distance between the axis, RFMH_2019_02_25
         # self.cog_distance = self.setup_parameter("~cog_distance", 0.0525)       #introduce the distance of turning point from the back axis (half the axis_distance), RFMH_2019_02_25
        self.limit = self.setup_parameter("~limit", 1.0)
        self.limit_max = 1.0
        self.limit_min = 0.0

        self.v_max = 999.0     # TODO: Calculate v_max !
        self.omega_max = 999.0     # TODO: Calculate v_max !

        # Prepare services
        self.srv_set_gain_dc = rospy.Service("~set_gain_dc", SetValue, self.cbSrvSetGainDc)             #adjust gain to dc-motor, RFMH_2019_02_25
        self.srv_set_trim_dc = rospy.Service("~set_trim_dc", SetValue, self.cbSrvSetTrimDc)             #adjust trim to dc-motor, RFMH_2019_02_25
         # self.srv_set_gain_servo = rospy.Service("~set_gain_servo", SetValue, self.cbSrvSetGainServo)    #create new service tor the callback function for gain_servo, RFMH_2019_02_25
         # self.srv_set_trim_servo = rospy.Service("~set_trim_servo", SetValue, self.cbSrvSetTrimServo)    #create new service for the callback function for trim_servo, RFMH_2019_02_25
        self.srv_set_baseline = rospy.Service("~set_baseline", SetValue, self.cbSrvSetBaseline)
        self.srv_set_radius = rospy.Service("~set_radius", SetValue, self.cbSrvSetRadius)
        self.srv_set_k = rospy.Service("~set_k", SetValue, self.cbSrvSetK)
         # self.srv_set_axis_distance = rospy.Service("~set_axis_distance", SetValue, self.cbSrvSetAxisDistance)       #create new service for the callback function for axis_distance, RFMH_2019_02_25
         # self.srv_set_cog_distance = rospy.Service("~set_cog_distance", SetValue, self.cbSrvSetCogDistance)          #create new service for the callback function for axis_distance, RFMH_2019_02_25
        self.srv_set_limit = rospy.Service("~set_limit", SetValue, self.cbSrvSetLimit)
        self.srv_save = rospy.Service("~save_calibration", Empty, self.cbSrvSaveCalibration)

        # Setup the publisher and subscriber
        self.sub_car_cmd = rospy.Subscriber("~car_cmd", Twist2DStamped, self.car_cmd_callback)
        self.sub_actuator_limits_received = rospy.Subscriber("~actuator_limits_received", BoolStamped, self.updateActuatorLimitsReceived, queue_size=1)
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd_dbv2", WheelsCmdDBV2Stamped, queue_size=1)
        self.pub_actuator_limits = rospy.Publisher("~actuator_limits", Twist2DStamped, queue_size=1)

        self.msg_actuator_limits = Twist2DStamped()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.actuator_limits_received = False
        self.pub_actuator_limits.publish(self.msg_actuator_limits)

        rospy.loginfo("[%s] Initialized.", self.node_name)
        self.printValues()

    def readParamFromFile(self):
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use default.yaml if file doesn't exsit
        if not os.path.isfile(fname):
            rospy.logwarn("[%s] %s does not exist. Using default.yaml." %(self.node_name,fname))
            fname = self.getFilePath("default")

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" %(self.node_name, fname, exc))
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["gain_dc", "trim_dc", "baseline", "radius", "k", "limit"]:
         #for param_name in ["gain_dc", "trim_dc", "gain_servo", "trim_servo", "baseline", "radius", "k", "axis_distance", "cog_distance", "limit"]:        #inserted the new parameters defined above, RFMH_2019_02_25
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~"+param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass

    def getFilePath(self, name):
        return (get_duckiefleet_root()+'/calibrations/kinematics/' + name + ".yaml")

    def updateActuatorLimitsReceived(self, msg_actuator_limits_received):
        self.actuator_limits_received = msg_actuator_limits_received.data

    def saveCalibration(self):
        # Write to yaml
        data = {
            "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
            "gain_dc": self.gain_dc,                                            #the new parameters for the dc-motor and the serve need to be written to the file as well
            "trim_dc": self.trim_dc,
             # "gain_servo": self.gain_servo,
             # "trim_servo": self.trim_servo,
            "baseline": self.baseline,
            "radius": self.radius,
            "k": self.k,
             # "axis_distance": self.axis_distance,                                #the two new parameters must be written to the yaml-file again, RFMH_2019_02_25
             # "cog_distance": self.cog_distance,
            "limit": self.limit,
        }

        # Write to file
        file_name = self.getFilePath(self.veh_name)
        with open(file_name, 'w') as outfile:
            outfile.write(yaml.dump(data, default_flow_style=False))
        # Printout
        self.printValues()
        rospy.loginfo("[%s] Saved to %s" %(self.node_name, file_name))

    def cbSrvSaveCalibration(self, req):
        self.saveCalibration()
        return EmptyResponse()

    def cbSrvSetGainDc(self, req):                                              #adjusted method name to "cbSrvSetGainDc", RFMH_2019_02_25
        self.gain_dc = req.value                                                #adjusted gain name to "gain_dc", RFMH_2019_02_25
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()

    def cbSrvSetTrimDc(self, req):                                              #adjusted method name to "cbSrvSetTrimDc", RFMH_2019_02_25
        self.trim_dc = req.value                                                #adjusted trim name to "trim_dc", RFMH_2019_02_25
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()

    # def cbSrvSetGainServo(self, req):
    #     self.gain_servo = req.value
    #     self.printValues()
    #     self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
    #     self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
    #     self.pub_actuator_limits.publish(self.msg_actuator_limits)
    #     return SetValueResponse()
    #
    # def cbSrvSetTrimServo(self, req):
    #     self.trim_servo = req.value
    #     self.printValues()
    #     self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
    #     self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
    #     self.pub_actuator_limits.publish(self.msg_actuator_limits)
    #     return SetValueResponse()

    def cbSrvSetBaseline(self, req):
        self.baseline = req.value
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()

    def cbSrvSetRadius(self, req):
        self.radius = req.value
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()

    def cbSrvSetK(self, req):
        self.k = req.value
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()

    # def cbSrvSetAxisDistance(self, req):                                        #Created two new methods for the parameters "axis_distance" and "cog_distance" to be set. RFMH_2019_02_25
    #     self.axis_distance = req.value
    #     self.printValues()
    #     self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
    #     self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
    #     self.pub_actuator_limits.publish(self.msg_actuator_limits)
    #     return SetValueResponse()
    #
    # def cbSrvSetCogDistance(self, req):
    #     self.cog_distance = req.value
    #     self.printValues()
    #     self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
    #     self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
    #     self.pub_actuator_limits.publish(self.msg_actuator_limits)
    #     return SetValueResponse()

    def cbSrvSetLimit(self, req):
        self.limit = self.setLimit(req.value)
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()

    def setLimit(self, value):
        if value > self.limit_max:
            rospy.logwarn("[%s] limit (%s) larger than max at %s" % (self.node_name, value, self.limit_max))
            limit = self.limit_max
        elif value < self.limit_min:
            rospy.logwarn("[%s] limit (%s) smaller than allowable min at %s" % (self.node_name, value, self.limit_min))
            limit = self.limit_min
        else:
            limit = value
        return limit

    def printValues(self):                                                      #adjust the output to all the new values as well in the log info, RFMH_2019_02_25
        rospy.loginfo("[%s] gain_dc: %s trim_dc: %s baseline: %s radius: %s k: %s limit: %s" % (self.node_name, self.gain_dc, self.trim_dc, self.baseline, self.radius, self.k, self.limit))
         #rospy.loginfo("[%s] gain_dc: %s trim_dc: %s gain_servo: %s trim_servo: %s baseline: %s radius: %s k: %s axis_distance: %s cog_distance: %s limit: %s" % (self.node_name, self.gain_dc, self.trim_dc, self.gain_servo, self.trim_servo, self.baseline, self.radius, self.k, self.axis_distance, self.cog_distance, self.limit))
    def car_cmd_callback(self, msg_car_cmd):
        if not self.actuator_limits_received:
            self.pub_actuator_limits.publish(self.msg_actuator_limits)

        # assuming same motor constants k for both motors
        k_wheel = self.k                                                        #changed "k_r" to "k_wheel", because there is only one such konstant need now, RFMH_2019_02_25
        k_l = self.k

        # adjusting k by gain and trim
        k_wheel_inv = (self.gain + self.trim) / k_wheel                             #changed "k_r" to "k_wheel", RFMH_2019_02_25
        k_l_inv = (self.gain - self.trim) / k_l

        omega_wheel = msg_car_cmd.v / self.radius                               #omega_r changed to omega_wheel and skipped the whole calculation
        gamma = pow(pow((msg_car_cmd.v / msg_car_cmd.omega),2.0) - pow(self.cog_distance,2.0),-0.5) * self.axis_distance     #omega_l changed to gamma as this is the steering angle and inserted the new calculation: gamma = f(v, omega)

        # conversion from motor rotation rate to duty cycle
        # u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)
        u_wheel = omega_wheel * k_wheel_inv                                             #omega_r changed to omega_wheel, u_r to u_wheel and k_r_inv to k_wheel_inv, RFMH_2019_02_25
        # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
        u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_wheel_limited = max(min(u_wheel, self.limit), -self.limit)            #u_r_limited changed to "u_wheel_limited" and "u_r" changed to "u_wheel", RFMH_2019_02_25
        u_l_limited = max(min(u_l, self.limit), -self.limit)

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdDBV2Stamped()
        msg_wheels_cmd.header.stamp = msg_car_cmd.header.stamp
        msg_wheels_cmd.vel_right = u_wheel_limited                              #vel_right is defined in the WheelsCmdDBV2Stamped --> name needs to be changed everywhere!, RFMH_2019_02_25
        msg_wheels_cmd.vel_left = u_l_limited
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == '__main__':
    rospy.init_node('inverse_kinematics_node', anonymous=False)
    inverse_kinematics_node = InverseKinematicsNode()
    rospy.spin()
