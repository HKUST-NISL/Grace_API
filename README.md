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

        Topic name: /hr/perception/hear/words

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
        The classifier uses 8-class emotion lable as output, i.e., {'Anger', 'Contempt', 'Disgust', 'Fear', 'Happiness', 'Neutral', 'Sadness', 'Surprise'}. However, in the output, {"Disgust", "Fear"} are relabled as "Agitation". 






### (5) Grace Behavior Change: Flags published to a ros topic upon the change of the behavior of Grace

        Topic name: /grace_proj/grace_behavior_change

        Message Type: std_msgs/String

        Message Format:

            string data #Text indicating what change just occurred

Note: 
    Possible state change of Grace: {"Speech Started", "Speech Stopped", "Speech Interrupted"}. In particular, upon the interruption of an on-going speech by her interlocutor, Grace will stop her speech, publish this state change flat, and wait for further commands.




### (6) Flow Control Commands: Flags published to a ros topic by the GUI, which is operated by the developer to control the interaction.

        Topic name: /grace_proj/start

        Message Type: std_msgs/Bool

        Message Format:

            bool data #When its value is true it means that all preparations like registering the target person is done and Grace should initialize the interaction sequence.


        Topic name: /grace_proj/stop

        Message Type: std_msgs/Bool

        Message Format:

            bool data #When its value is true it means that we would like Grace to stop the current interaction sequence as some error occurred.









## Input specification:














































