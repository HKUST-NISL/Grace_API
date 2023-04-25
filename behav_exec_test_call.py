import yaml
import rospy
import os
import re
import threading

import dynamic_reconfigure.client
import sensor_msgs.msg
import std_msgs.msg
import hr_msgs.msg
import grace_attn_msgs.msg
import grace_attn_msgs.srv
import hr_msgs.msg
import hr_msgs.cfg
import hr_msgs.srv
import std_msgs



#Load configs
def loadConfigs():
    #Load configs
    with open("./Configs/config.yaml", "r") as config_file:
        grace_api_configs = yaml.load(config_file, Loader=yaml.FullLoader)
        print("Read successful")
    return grace_api_configs

grace_api_configs = loadConfigs()


if __name__ == '__main__':
    #Ros routine
    rospy.init_node("exec_test")
    grace_behavior_client = rospy.ServiceProxy(grace_api_configs['Ros']['grace_behavior_service'], grace_attn_msgs.srv.GraceBehavior)

    #Prepare a service request
    req = grace_attn_msgs.srv.GraceBehaviorRequest()
    req.command = grace_api_configs['Behavior']['behav_exec_cmd']
    req.utterance = grace_api_configs['Debug']['Sample']['txt']
    req.lang = grace_api_configs['Debug']['Sample']['lang']

    req.expressions = grace_api_configs['Debug']['Sample']['expressions']
    req.exp_start = grace_api_configs['Debug']['Sample']['exp_start']
    req.exp_end = grace_api_configs['Debug']['Sample']['exp_end']
    req.exp_mag = grace_api_configs['Debug']['Sample']['exp_mag']

    req.gestures = grace_api_configs['Debug']['Sample']['gestures']
    req.ges_start = grace_api_configs['Debug']['Sample']['ges_start']
    req.ges_end = grace_api_configs['Debug']['Sample']['ges_end']
    req.ges_mag = grace_api_configs['Debug']['Sample']['ges_mag']

    #Call the service
    print("Service call response is:\n %s" % grace_behavior_client(req))






