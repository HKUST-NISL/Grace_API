import yaml
import rospy
import os
import re
import threading
from signal import signal
from signal import SIGINT

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
    with open("./config/config.yaml", "r") as config_file:
        grace_api_configs = yaml.load(config_file, Loader=yaml.FullLoader)
        print("Read successful")
    return grace_api_configs

grace_api_configs = loadConfigs()


class GraceAPI:

    __latest_word = ''#Not used

    __start_faking = False
    __latest_interim = None
    __latest_interim_time_stamp = 0
    __latest_interim_for_bardging_in = ''

    __latest_tts_event = ''
    __behav_service_thread_keep_alive = True
    __bardging_handling_on = True


    def __init__(self):

        self.__initRosConnection()

        self.__mainLoop()


    def __mainLoop(self):
        rate = rospy.Rate(0.1)
        while(True):
            # self.__triggerExpressionFixedDur('happy',3,0.8)
            rate.sleep()


    def __initRosConnection(self):
        rospy.init_node(grace_api_configs['Ros']['node_name'])
        
        #For configuring and monitoring asr
        self.asr_words_sub = rospy.Subscriber(grace_api_configs['Ros']['asr_words_topic'], hr_msgs.msg.ChatMessage, self.__asrWordsCallback, queue_size=grace_api_configs['Ros']['queue_size'])
        self.asr_interim_sub = rospy.Subscriber(grace_api_configs['Ros']['asr_interim_speech_topic'], hr_msgs.msg.ChatMessage, self.__asrInterimCallback, queue_size=grace_api_configs['Ros']['queue_size'])
        self.asr_fake_sentence_pub = rospy.Publisher(grace_api_configs['Ros']['asr_fake_sentence_topic'], hr_msgs.msg.ChatMessage, queue_size=grace_api_configs['Ros']['queue_size'])
        self.asr_reconfig_client = dynamic_reconfigure.client.Client(grace_api_configs['Ros']['asr_reconfig']) 
        self.__asrInit()

        #For tts
        self.tts_data_client = rospy.ServiceProxy(grace_api_configs['Ros']['tts_data_service'], hr_msgs.srv.TTSData)
        self.tts_say_client = rospy.ServiceProxy(grace_api_configs['Ros']['tts_say_service'], hr_msgs.srv.TTSTrigger)
        self.tts_control_pub = rospy.Publisher(grace_api_configs['Ros']['tts_control_topic'], std_msgs.msg.String, queue_size=grace_api_configs['Ros']['queue_size'])
        self.tts_event_sub = rospy.Subscriber(grace_api_configs['Ros']['tts_event_topic'], std_msgs.msg.String, self.__ttsEventCallback, queue_size=grace_api_configs['Ros']['queue_size'])

        #For arm gesture
        self.arm_animation_pub = rospy.Publisher(grace_api_configs['Ros']['arm_animation_topic'], hr_msgs.msg.SetAnimation, queue_size=grace_api_configs['Ros']['queue_size'])
        self.arm_animation_normal_length = rospy.ServiceProxy(grace_api_configs['Ros']['arm_animation_normal_length_service'], hr_msgs.srv.GetAnimationLength)
        self.arm_animation_reconfig_client = dynamic_reconfigure.client.Client(grace_api_configs['Ros']['arm_animation_motor_speed_reconfig']) 
        self.__configAnimationMotorSpeed(grace_api_configs['Behavior']['arm_anim_motor_transition_time'])

        #For factial expression
        self.expression_pub = rospy.Publisher(grace_api_configs['Ros']['expression_topic'], hr_msgs.msg.SetExpression, queue_size=grace_api_configs['Ros']['queue_size'])

        #For accepting behavioural commands
        self.end_of_conv_sub = rospy.Subscriber(grace_api_configs['Ros']['end_of_conv_topic'], std_msgs.msg.Bool, self.__endOfConvCallback, queue_size=grace_api_configs['Ros']['queue_size'])
        self.grace_behavior_server = rospy.Service(grace_api_configs['Ros']['grace_behavior_service'], grace_attn_msgs.srv.GraceBehavior, self.__handleGraceBehaviorServiceCall)

        #Some debug topic 
        self.bardging_switch_sub = rospy.Subscriber(grace_api_configs['Debug']['bardging_in_switch_topic'],std_msgs.msg.Bool, self.__bardgingHandlingSwitchCallback, queue_size=grace_api_configs['Ros']['queue_size'])

        #tmp camera angle
        self.__cam_ang_sub = rospy.Subscriber("/grace_proj/set_cam_angle", std_msgs.msg.Float32, self.__cameraAngCallback, queue_size=grace_api_configs['Ros']['queue_size'])
        self.__cam_cfg_client = dynamic_reconfigure.client.Client('/hr/perception/camera_angle')

    '''
    #   ASR-ROS-Helpers
    '''
    def __asrInit(self):
        #We turn off first
        params = { 'enable': False} 
        self.asr_reconfig_client.update_configuration(params)
        #Then restart
        params = { 'enable': True, 'language': grace_api_configs['Ros']['primary_language_code'], 'alternative_language_codes': grace_api_configs['Ros']['secondary_language_code'], 'model': grace_api_configs['Ros']['asr_model'], 'continuous': grace_api_configs['Ros']['asr_continuous']} 
        self.asr_reconfig_client.update_configuration(params)
        #Start the fake sentence generator
        self.__fake_sentence_thread = threading.Thread(target = self.__fakeSentenceThread, daemon=False)
        self.__fake_sentence_thread.start()



    def __reconfigArmAnimTransitionSpeed(self, speed):
        pass

    def __asrWordsCallback(self, msg):
        self.__latest_word = msg.utterance
        print('Latest WORD: (%s).' % self.__latest_word)

    def __asrInterimCallback(self, msg):
        #Receive the latest asr string
        self.__latest_interim = msg
        self.__latest_interim_for_bardging_in = self.__latest_interim.utterance
        print('Latest INTERIM %s' %{self.__latest_interim.utterance})

        #Upon receiving a new interim sentence, we update the timestamp and start faking sentences
        self.__start_faking = True
        self.__latest_interim_time_stamp = rospy.get_time()
        

    def __fakeSentenceThread(self):
        rate = rospy.Rate(grace_api_configs['Ros']['asr_fake_sentence_check_rate'])

        while True:
            rate.sleep()

            if( self.__start_faking ):#If we have started to fake a sentence
                #Check the timestamp of the latest interim speech
                if( rospy.get_time() - self.__latest_interim_time_stamp >= grace_api_configs['Ros']['asr_fake_sentence_window'] ):
                    #Publish a fake sentence  
                    self.asr_fake_sentence_pub.publish(self.__latest_interim)

                    #Log
                    print('Publishing FAKE asr sentence %s' % {self.__latest_interim.utterance})

                    #Reset the fields
                    self.__start_faking = False
                    self.__latest_interim = None






    '''
    #   TTS-ROS-Helpers
    '''
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

    def __ttsEventCallback(self, msg):
        self.__latest_tts_event = msg.data
        print('Latest TTS event is \"%s\".' % self.__latest_tts_event)



    '''
    #   Gesture-ROS-Helpers
    '''
    def __configAnimationMotorSpeed(self, state_transition_dur):
        #This is the state transition time for the motor to achieve when going from one key frame to another key frame

        #Compare with min state transition for safety
        if(state_transition_dur < grace_api_configs['Behavior']['arm_anim_min_motor_transition_time'] ):
            state_transition_dur_rectified = grace_api_configs['Behavior']['arm_anim_min_motor_transition_time'] 
        else:
            state_transition_dur_rectified = state_transition_dur

        #Reconfigure HRSDK
        params = { 'arm_animation_transition': state_transition_dur_rectified } 
        self.arm_animation_reconfig_client.update_configuration(params)



    def __queryArmAnimationNormalLength(self, name):
        #Compose a request
        req = hr_msgs.srv.GetAnimationLengthRequest()
        req.animation = name

        res = self.arm_animation_normal_length(req)

        #Return the normal playback duration at playback speed 1
        return res.length

    def __calcArmAnimationPlaybackSpeed(self, normal_dur, dur_in):
        #The playback duration should be NO LESS than motor state transition duration
        if(dur_in < grace_api_configs['Behavior']['arm_anim_min_motor_transition_time'] ):
            dur_rectified = grace_api_configs['Behavior']['arm_anim_min_motor_transition_time'] 
        else:
            dur_rectified = dur_in
        #Compute playback speed that will achieve this duration
        playback_speed_ratio = normal_dur / dur_rectified

        return playback_speed_ratio


    def __triggerArmAnimation(self, name, speed, magnitude):
        #Compose a message
        msg = hr_msgs.msg.SetAnimation()
        msg.name = name
        msg.speed = speed
        msg.magnitude = magnitude

        #Publish
        self.arm_animation_pub.publish(msg)


    def __triggerArmAnimationFixedDur(self, name, dur, magnitude):
        #Normal duration with playback speed 1 - NOT MOTOR SPEED
        normal_dur = self.__queryArmAnimationNormalLength(name)

        #Compute the speed to reach the animation duration specified in the input
        playback_speed_ratio = self.__calcArmAnimationPlaybackSpeed(normal_dur, dur)

        self.__triggerArmAnimation(str(name), playback_speed_ratio, magnitude)




    '''
    #   Expression-ROS-Helpers
    '''
    def __triggerExpressionFixedDur(self, name, dur, magnitude):
        #Compose a message
        msg = hr_msgs.msg.SetExpression()
        msg.name = str(name)
        msg.duration = rospy.Duration(dur,0)
        msg.magnitude = float(magnitude)

        #Publish
        self.expression_pub.publish(msg)



    #Camear angle
    def __cameraAngCallback(self, msg):
        self.setCameraAngle(msg.data)
        
    def setCameraAngle(self, angle):
		#tilt chest cam to a given angle
        try:
            print("Configuring camera angle to %f." % angle)

            self.__cam_cfg_client.update_configuration({"motor_angle":angle})
        except Exception as e:
            print(e)


    '''
    #   Interface
    '''

    def __bardgingHandlingSwitchCallback(self, msg):
        self.__bardging_handling_on = msg.data
        print("Barding handling is %s" % self.__bardging_handling_on)


    def __handleGraceBehaviorServiceCall(self, req):
        #Prepare response object
        res = grace_attn_msgs.srv.GraceBehaviorResponse()

        if(req.command == grace_api_configs['Behavior']['behav_exec_cmd']):
            res = self.__execBehavior(req,res)
        elif(req.command == grace_api_configs['Behavior']['behav_stop_cmd']):
            res = self.__stopBehavior(req,res)
        else:
            print("Unexpected behavior command %s." % req.command)

        return res


    def __stopBehavior(self, req, res):
        self.__stopAllBehviors()
        res.result = grace_api_configs['Behavior']['behav_stopped_string']
        return res


    def __execBehavior(self, req, res):
        #We don't need auto-generated expressions and gestures anymore
        pure_text = grace_api_configs['Ros']['tts_pure_token'] + req.utterance
        
        #Get total duration of tts
        dur_total = self.__parseTTSDur(self.__getTTSMeta(pure_text, req.lang))

        #Arrange expressions and gestures in physical time
        expression_seq = self.__arrangeBehavSeq(dur_total, req.expressions, req.exp_start, req.exp_end, req.exp_mag)
        gesture_seq = self.__arrangeBehavSeq(dur_total, req.gestures, req.ges_start, req.ges_end, req.ges_mag)

        #Prepare two threads for executing expression and gestures
        self.__behav_service_thread_keep_alive = True
        exp_thread = threading.Thread(target=lambda: self.__execBehavSeq(expression_seq, self.__triggerExpressionFixedDur), daemon=False)
        ges_thread = threading.Thread(target=lambda: self.__execBehavSeq(gesture_seq, self.__triggerArmAnimationFixedDur), daemon=False)

        #Will poll the tts event for flow control and the asr input in case there is any bardging in behavior
        rate = rospy.Rate(grace_api_configs['Behavior']['bardging_in_monitor_rate'])
        self.__latest_interim_for_bardging_in = ''
        self.__latest_tts_event = ''

        #Initiate tts, gesture, expression and start polling
        self.behavior_exec_start_time = rospy.get_time()
        self.__say(pure_text, req.lang)
        exp_thread.start()
        ges_thread.start()
        while True:
            rate.sleep()
            if(self.__bardging_handling_on and self.__latest_interim_for_bardging_in):#Someone said somthing when Grace is performing
                print('Bardging in detected due to interim %s!' % self.__latest_interim_for_bardging_in)

                #Stop behavior command execution completely
                self.__stopAllBehviors()

                #Report bardging in
                res.result = grace_api_configs['Behavior']['behav_bardging_string']

                #Break the loop and finish the service
                break
            
            else:#Nobody said anything, check the tts state
                if(self.__latest_tts_event == grace_api_configs['Ros']['tts_end_event']):#TTS is over
                    print('Successfully completed!')
                    
                    #Stop gesture and expressions
                    self.__goToNeutral()

                    #Report successful completion of the behaviour execution
                    res.result = grace_api_configs['Behavior']['behav_succ_string']

                    #Break the loop and finish the service
                    break

                else:#TTS still going
                    pass#Do nothing
        return res


    def __arrangeBehavSeq(self, total_dur, names, start_portion, end_portion, magnitude):
        num_behav = len(names)


        behav_seq = [None] * num_behav
        for i in range(num_behav):
            behav_seq[i] = [None] * 4 


        for i in range(num_behav):
            behav_seq[i][0] = names[i]
            behav_seq[i][1] = start_portion[i] * total_dur
            behav_seq[i][2] = end_portion[i] * total_dur
            behav_seq[i][3] = magnitude[i]
        return behav_seq


    def __execBehavSeq(self, behav_seq, exec_fnc):
        
        num_behav = len(behav_seq)
        
        rate = rospy.Rate(grace_api_configs['Behavior']['behav_exec_rate'])

        #The behaviour to be executed
        exec_cnt = 0
        #Total time since the start of thie performance command
        elapsed_time = 0


        while self.__behav_service_thread_keep_alive:
            #Update the elapsed time
            elapsed_time = rospy.get_time() - self.behavior_exec_start_time

            if( exec_cnt < num_behav):# Start executing this behavior
                if( elapsed_time >= behav_seq[exec_cnt][1]):
                    print("Executing behavior %d: %s" % (exec_cnt , behav_seq[exec_cnt][0]))

                    print(type(behav_seq[exec_cnt][0]))
                    exec_fnc(behav_seq[exec_cnt][0], behav_seq[exec_cnt][2] - behav_seq[exec_cnt][1], behav_seq[exec_cnt][3])
                    
                    exec_cnt = exec_cnt + 1 
            else:#Nothing more to execute
                break

            rate.sleep()









    def __endOfConvCallback(self, msg):
        self.__stopAllBehviors()



    def __stopAllBehviors(self):
        #Cutoff any on-going tts
        self.__stopTTS()
        #Reset to neutral arm-pose and facial expression
        self.__goToNeutral()


    def __goToNeutral(self):
        #Kill any on-going behaviour service thread
        self.__behav_service_thread_keep_alive = False

        #Reset to a neutral arm pose
        self.__triggerArmAnimationFixedDur(grace_api_configs['Behavior']['neutral_pose_info']['name'],grace_api_configs['Behavior']['neutral_pose_info']['dur'],grace_api_configs['Behavior']['neutral_pose_info']['magnitude'])

        #Reset to a neutral expression
        self.__triggerExpressionFixedDur(grace_api_configs['Behavior']['neutral_expression_info']['name'],grace_api_configs['Behavior']['neutral_expression_info']['dur'],grace_api_configs['Behavior']['neutral_expression_info']['magnitude'])
        

def handle_sigint(signalnum, frame):
    # terminate
    print('Main interrupted! Exiting.')
    sys.exit()

if __name__ == '__main__':

    signal(SIGINT, handle_sigint)


    grace_api = GraceAPI()



















