# Grace_API


Here are the specifications of the interface between the dialogue_manager and the primitive functions of Grace & high-level controls from the GUI.


## Output specification: 

### (1) ASR Sentence: Complete sentence reported by the asr module of Grace published to a ros topic 

        Topic name: /hr/perception/hear/sentence 

        Message Type: hr_msgs/ChatMessage 

        Message Format: 

            string utterance    #This is the text string of the sentence 

            string lang         #Language code 
        
            uint8 confidence    #Confidence level reported by ASR 

            string source       #Provider of the ASR service 

            string audio_path   #Cached audio file path 

Sample: \
    ![sentence sample](/Images/sentence_sample.png)


### (2) ASR Stream (Word): Word stream reported by the asr module of Grace published to a ros topic - note that this stream could be unstable as ASR module is dynamically revising its judgements on words

        Topic name: /hr/perception/hear/words

        Message Type: hr_msgs/ChatMessage

        Message Format:

            string utterance    #This is the text string of an individual word 

            string lang         #Language code

            uint8 confidence    #Confidence level reported by ASR

            string source       #Provider of the ASR service

            string audio_path   #Cached audio file path 

Sample: (Note the revision of the word "see")\
    ![word sample](/Images/word_sample.png)



### (3) ASR Stream (Sentence): Incomplete sentence stream reported by the asr module of Grace published to a ros topic - note that this stream grows as the ASR module listens and make judgements.

        Topic name: /hr/perception/hear/interim_speech

        Message Type: hr_msgs/ChatMessage

        Message Format:

            string utterance    #This is the text string of the (incomplete) sentence 

            string lang         #Language code

            uint8 confidence    #Confidence level reported by ASR

            string source       #Provider of the ASR service

            string audio_path   #Cached audio file path 

Sample: (Note the growing, incomplete sentence)\
    ![interim speech](/Images/interim_speech_sample.png)




### (4) Emotion Recognition: Instantaneous stream of the expression & attention label of the target person published to a ros topic.

        Topic name: /grace_proj/emotion_attention_target_person_output_topic

        Message Type: grace_attn_msgs/EmotionAttentionResult

        Message Format:

            std_msgs/String attention

                string data #This is a string label indicating whether the person is paying attention, i.e., looking at Grace

            std_msgs/String emotion

                string data #This is a string label indicating the instantaneous expression of the person

            sensor_msgs/Image visualization_frame #Face image annotated with target person's gaze direction, emotion label and confidence score.

                std_msgs/Header header

                    uint32 seq

                    time stamp

                    string frame_id

                uint32 height

                uint32 width

                string encoding

                uint8 is_bigendian

                uint32 step

                uint8[] data
Note: 

    Possible attention label:
        "True" or "False"

    Possible emotion label:
        The classifier uses 8-class emotion lable as output, i.e., {'Anger', 'Contempt', 'Disgust', 'Fear', 'Happiness', 'Neutral', 'Sadness', 'Surprise'}. However, in the output, {"Disgust", "Fear"} are relabled as "Agitation". If the target person's face is not detected, "Abscence" would be placed in the "emotion" field. When the target person is not tracked at all, "EXCEPTION!!" would be placed in the "emotion" field.



### (5) Flow Control Commands: Flags published to ros topics by the GUI, which is operated by the developer to control the interaction.

        Topic name: /grace_proj/start

        Message Type: std_msgs/Bool

        Message Format:

            bool data #When its value is true it means that all preparations like registering the target person is done and Grace should initialize the interaction sequence.


        Topic name: /grace_proj/stop

        Message Type: std_msgs/Bool

        Message Format:

            bool data #When its value is true it means that we would like Grace to stop the current interaction sequence as some error occurred.









## Input specification:


### (1) Grace Behavior Command Service: A ROS service that makes Grace exeute a particular behavior as specified. 

        Service Name: /grace_proj/execute_behavior

        Input Args:
            string  command      #Text indicating the nature of this behavior command, for now we don't worry about this
            string  utterance    #Text indicating the sentence Grace should say
            string  lang         #Language code

            string[] expressions #The sequence of facial expressions that should accompany the utterance
            float32[] exp_start  #Start time (relative) of the facial expressions
            float32[] exp_end    #End time (relative) of the facial expressions
            float32[] exp_mag    #The magnitude of expression, between 0 and 1

            string[] gestures    #The sequence of gestures that should accompany the utterance
            float32[] ges_start  #Start time (relative) of the gestures
            float32[] ges_end    #Start time (relative) of the gestures
            float32[] ges_mag    #The magnitude of arm animation, between 0 and 1


        Return Args:
            string result        #Text indicating the outcome of this behavior execution

Note: 

    "command" could either be
    (1) "exec", meaning to execute a defined behavior
    or (2) "stop", meaning to force stop any on-going behaviors


    "result" could either be 

    (1) "completed", meaning that Grace completed the execution without noticing any interruption 
    (2) "interrupted", meaning that Grace was interruped by her interlocutor and has to stop the speech execution as a reaction mid-way.
    or(3) "stopped", meaning the stop behavior command has been executed.


### (2) Grace Keepalive Behavior Command: Command that controls fallback behaviors like nodding and aversion published to ros topics


        Topic name: /grace_proj/toggle_nodding_topic

        Message Type: std_msgs/Bool

        Message Format:

            bool data #When its value is true Grace will nod once using a randomly picked nod animation


        Topic name: /grace_proj/toggle_aversion

        Message Type: std_msgs/Bool

        Message Format:

            bool data #When its value is true Grace will start to have aversions on a random basis
































