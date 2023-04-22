# Grace_API



## Output specification: 

### (1) ASR Sentence: complete sentence reported by the asr module of Grace published to a ros topic 

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


### (2) ASR Stream (Word): word stream reported by the asr module of Grace published to a ros topic - note that this stream could be unstable as ASR module is dynamically revising its judgements on words

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




### (4) Emotion Recognition: instantaneous expression & attention label of the target person 

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
Sample: 

    Possible attention label:
        "True" or "False"
        
    Possible emotion label:
        The classifier uses 8-class emotion lable as output, i.e., {'Anger', 'Contempt', 'Disgust', 'Fear', 'Happiness', 'Neutral', 'Sadness', 'Surprise'}. However, in the output, {"Disgust", "Fear"} are relabled as "Agitation". 











## Input specification:














































