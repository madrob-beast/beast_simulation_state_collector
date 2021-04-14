#!/usr/bin/env python

import rospy
import roslaunch
import rospkg
import os
import sys
import numpy as np

from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from urdf_parser_py.urdf import URDF
from std_msgs.msg import Float64
from beast_simulation_state_collector.srv import BeastCartDummy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetModelProperties
from gazebo_msgs.srv import GetJointProperties
from beast_srvs.srv import *
from beast_msgs.msg import *
from eurobench_bms_msgs_and_srvs.srv import *
from sensor_msgs.msg import JointState



VERBOSE = False
START_SIM = False

class beast_simulation_state_collector:
    def __init__(self):
        self.current_benchmark_name = None
        self.old_benchmark_name = None
        self.left_position_old = 0.0
        self.right_position_old = 0.0
        self.left_position = 0.0
        self.right_position = 0.0
        
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.last_time_in_sec = 0.0        
        
        self.left_wheel_status_publisher = rospy.Publisher('/beast_cart/left/wheel_status',
                                        Wheel, queue_size=1)            
                                                         
        self.right_wheel_status_publisher = rospy.Publisher('/beast_cart/right/wheel_status',
                                        Wheel, queue_size=1)                    
        
        if START_SIM:
             self.startSim()           
          
    def startSim(self):
        package = 'eurobench_reemc_cart'
        launch_file = 'reemc_cart.launch'
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        
        launch_file = os.path.join(rospkg.RosPack().get_path(package), 'launch', launch_file)
     #   sys.argv = [ 'door:=simple', 'direction:=push', 'robot_placement_cw:=true']
        
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        self.launch.start()
        
    def are_ranges_complete(self, ranges):
        for single_range in ranges:
            if single_range == None:
                return False
        return True
        
    def null_the_ranges(self, ranges):
        for single_range in ranges:
            single_range = None
        
    def sensor_identifier(self, ros_data):
        return int(ros_data.header.frame_id[-1])%4

    
def createWheelMsgs(ebws):
    msg_left = Wheel()
    msg_right = Wheel()
    
    msg_left.braking_force = 0
    msg_left.velocity = ebws.left_vel
    msg_right.braking_force = 0
    msg_right.velocity = ebws.right_vel
    
    return msg_left, msg_right    


def updateWheelVelocities(ebws):
    try:
        get_wheel_joint_props = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    if VERBOSE: print('---------- joint vel ---------')
    joint_prop = get_wheel_joint_props('wheel_rear_left_spin')
    if VERBOSE: print(joint_prop.position[0])
    ebws.left_position = joint_prop.position[0]
    
    joint_prop = get_wheel_joint_props('wheel_rear_right_spin')
    if VERBOSE: print(joint_prop.position[0])
    ebws.right_position = joint_prop.position[0]
    
    current_time = rospy.get_time()
    d_time = current_time - ebws.last_time_in_sec
    if d_time == 0.0:
        return 0.0, 0.0
        
    ebws.left_vel = (ebws.left_position - ebws.left_position_old)/d_time
    ebws.right_vel = (ebws.right_position - ebws.right_position_old)/d_time    
    
    ebws.left_position_old = ebws.left_position     
    ebws.right_position_old = ebws.right_position 
    ebws.last_time_in_sec = current_time
    
    return ebws.left_vel, ebws.right_vel


       
def talker(ebws):
    r = rospy.Rate(30) 
    
    while not rospy.is_shutdown():
        msg_handle = getTrolleyPosition()
    
        updateWheelVelocities(ebws)
        msg_left, msg_right = createWheelMsgs(ebws)
        
        ebws.left_wheel_status_publisher.publish(msg_left)
        ebws.right_wheel_status_publisher.publish(msg_right)        
        


        retrieveBenchmarkConfiguration(ebws)

        r.sleep()


def callback(data):
    rospy.loginfo("%s is age: %d" % (data.name, data.age))
    print ("initialized")

def handle_beast_trolley_dummy_srv(req):  #TODO CHECK VINCENZO
    print("Handled dummy service")
    return SetStiffness.srvResponse(True, "")

def listener(self):
    image_camera = rospy.Subscriber("sensor_msgs/Image", Image, callback)

def getTrolleyPosition():
    try:
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates('pushcart', '')
        if VERBOSE:
            print '\n'
            print 'Status.success = ', resp_coordinates.success
            print("---- Pushcart pose \n: " + str(resp_coordinates.pose.position))

    except rospy.ServiceException as e:
        rospy.loginfo("Get Model State service call failed:  {0}".format(e))
        
    #return joint_prop_handle.position[0]


def retrieveBenchmarkConfiguration(ebws):    # Based on the currently selected benchmark type
    try:
        #rospy.wait_for_service('beast/gui/benchmark_params')
        get_benchmark_params = rospy.ServiceProxy('beast/gui/benchmark_params', BeastBenchmarkParams)
    except rospy.ServiceException, e:
        print "ServiceProxy failed: %s"%e
        exit(0)
    response = get_benchmark_params()
    ebws.disturbance_type = response.disturbance_type
    ebws.start_already_gripping = response.start_already_gripping
    ebws.load = response.load
    if VERBOSE:
        print "\n---- trolley stiffness from gui:"
        print ebws.disturbance_type
        
        print "\n---- trolley start_already_gripping from gui:"
        print ebws.start_already_gripping
        
        print "\n---- trolley load from gui:"
        print ebws.load
        


def benchmarkConfigurationHasChanged(ebws):
    if (ebws.current_door_opening_side != ebws.old_door_opening_side
    or ebws.current_robot_approach_side != ebws.old_robot_approach_side):
        # to restart also when simulation force change use 
        # ebws.current_benchmark_name != ebws.old_benchmark_name
        ebws.old_benchmark_name = ebws.current_benchmark_name
        ebws.old_door_opening_side = ebws.current_door_opening_side
        ebws.old_robot_approach_side = ebws.current_robot_approach_side
        if VERBOSE: 
            print("PARAMETERS ARE CHANGED")
            
        return True;
    return False;
        
def noforce():
    return 'door:=simple','self_closing:=n'
 
def constant_force():
    return 'door:=simple','self_closing:=y'
 
def sudden_force():
    return 'door:=hard_obstacle','self_closing:=n'
 
def sudden_ramp():
    return 'door:=soft_obstacle','self_closing:=n'
 
def wind_ramp():
    return 'door:=wind','self_closing:=n'
         
def getScene(benchmark_name):
    scene_map = {
        "No Force": noforce,
        "Constant Force": constant_force,
        "Sudden Force": sudden_force,
        "Sudden Ramp": sudden_ramp,
        "Wind Ramp": wind_ramp,
    }
    func = scene_map.get(benchmark_name)
    arg1, arg2 = func()
    return arg1, arg2
    
def restartSim(ebws):
    print("***** RESTARTING SIMULATION FOR PARAMETER CHANGE *****")
    
    ebws.launch.shutdown()
    print ebws.current_benchmark_name
    # arg0, arg1 = getScene(ebws.current_benchmark_name)
    # now the force is controlled by the gazebo "simple door" plugin
    arg0, arg1 = 'door:=simple','self_closing:=n' 
    
    arg2 = 'direction:=pull' if ebws.current_door_opening_side == "CW" else 'direction:=push'
    arg3 = 'robot_placement_cw:=true' if ebws.current_robot_approach_side == "CW" else 'robot_placement_cw:=false'
    
    package = 'eurobench_reemc_door'
    launch_file = 'reemc_door.launch'
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch_file = os.path.join(rospkg.RosPack().get_path(package), 'launch', launch_file)
    sys.argv = [arg0, arg2, arg1, arg3]
    ebws.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    ebws.launch.start()


def main(args):
     ebws =  beast_simulation_state_collector()
     rospy.init_node('beast_simulation_state_collector', anonymous=True) 
     
     listener(ebws)

     s = rospy.Service('/beast_cart/reset_encoders', ResetEncoders, handle_beast_trolley_dummy_srv) 
     print ("service beast/trolley initialized in beast_simulation_state_collector")    

     try:
         talker(ebws)
         rospy.spin()
     except KeyboardInterrupt:
           print ("Shutting down ROS madrob_simulation_state_collector module")


if __name__ == '__main__':
     main(sys.argv)
