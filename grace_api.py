import yaml
import rospy
import os
import re

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


class GraceAPI:

    __latest_word = ''




    def __init__(self):

        self.__initRosConnection()

        rospy.spin()


    def __initRosConnection(self):
        rospy.init_node(grace_api_configs['Ros']['node_name'])
        
        #For configuring and monitoring asr
        self.asr_words_sub = rospy.Subscriber(grace_api_configs['Ros']['asr_words_topic'], hr_msgs.msg.ChatMessage, self.__asrWordsCallback, queue_size=grace_api_configs['Ros']['queue_size'])
        self.asr_reconfig_client = dynamic_reconfigure.client.Client(grace_api_configs['Ros']['asr_reconfig']) 
        self.__asrInit()

        #For tts
        self.tts_data_client = rospy.ServiceProxy(grace_api_configs['Ros']['tts_data_service'], hr_msgs.srv.TTSData)
        self.tts_say_client = rospy.ServiceProxy(grace_api_configs['Ros']['tts_say_service'], hr_msgs.srv.TTSTrigger)
        self.tts_control_pub = rospy.Publisher(grace_api_configs['Ros']['tts_control_topic'], std_msgs.msg.String, queue_size=grace_api_configs['Ros']['queue_size'])


        #For arm gesture
        self.arm_animation_pub = rospy.Publisher(grace_api_configs['Ros']['arm_animation_topic'], hr_msgs.msg.SetAnimation, queue_size=grace_api_configs['Ros']['queue_size'])
        self.arm_animation_reconfig_client = dynamic_reconfigure.client.Client(grace_api_configs['Ros']['arm_animation_speed_reconfig']) 

        #For factial expression
        self.expression_pub = rospy.Publisher(grace_api_configs['Ros']['expression_topic'], hr_msgs.msg.SetExpression, queue_size=grace_api_configs['Ros']['queue_size'])

        #For accepting behavioural commands
        self.end_of_conv_sub = rospy.Subscriber(grace_api_configs['Ros']['end_of_conv_topic'], std_msgs.msg.Bool, self.__endOfConvCallback, queue_size=grace_api_configs['Ros']['queue_size'])
        self.grace_behavior_server = rospy.Service(grace_api_configs['Ros']['grace_behavior_service'], grace_attn_msgs.srv.GraceBehavior, self.__handleGraceBehaviorServiceCall)


    def __asrInit(self):
        params = { 'enable': True, 'language': grace_api_configs['Ros']['cantonese_language_code'], 'alternative_language_codes': grace_api_configs['Ros']['english_language_code'], 'model': grace_api_configs['Ros']['asr_model'], 'continuous': True} 
        self.asr_reconfig_client.update_configuration(params)

    def __reconfigArmAnimTransitionSpeed(self, speed):
        pass


    def __asrWordsCallback(self, msg):
        __latest_word = msg.utterance
        print('Latest ASR word: (%s).' % __latest_word)

    def __getTTSMeta(self, text, lang = grace_api_configs['Ros']['english_language_code']):
        #Compose a request
        req = hr_msgs.srv.TTSDataRequest()
        req.txt = text
        req.lang = lang

        #Call the service
        res = self.tts_data_client(req)
        return res

    def __parseTTSDur(self, tts_data_response):
        #Only one occurence by default
        dur_text = re.search('\"duration\": [0123456789.]+,', tts_data_response.data)
        dur = float(re.search('[0123456789.]+',dur_text.group()).group())
        return dur

    def __pubTTSCtrlCmd(self, cmd):
        #Compose a message
        msg = std_msgs.msg.String()
        msg.data = cmd

        #Publish
        self.tts_control_pub.publish(msg)

    def __stopTTS(self):
        self.__pubTTSCtrlCmd(grace_api_configs['Ros']['tts_stop_cmd'])

    def __say(self, text, lang = grace_api_configs['Ros']['english_language_code']):
        #Compose a request
        req = hr_msgs.srv.TTSTriggerRequest()
        req.text = text
        req.lang = lang

        #Call the service
        return self.tts_say_client(req)



    def __handleGraceBehaviorServiceCall(self, req):
        pass

    def __endOfConvCallback(self, msg):
        pass




if __name__ == '__main__':
    grace_api = GraceAPI()



















