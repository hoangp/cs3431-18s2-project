#!/usr/bin/env python

import speech_recognition as sr 

class Commander: 

    def __init__(self, commands = None):
        if commands is None:
            self.commands = ['stop']
        self.commands = commands

    def publish_func(self, msg):
        print(msg)
        
    def lis_mic(self, STOP = 'stop'):
        
        r = sr.Recognizer()  # obtain audio from the microphone
        with sr.Microphone() as source:  
            print("Please wait. Calibrating microphone...")  
            r.adjust_for_ambient_noise(source, duration=3)  
            print("Say something!")  
            
            words = ''
            while words.find(STOP) == -1:
               
                audio = r.listen(source, phrase_time_limit=5)  
                   
                try:   # recognize speech using Sphinx 
                    words = r.recognize_google(audio)
                    print('({})'.format(words))
                    for c in self.commands:
                        if words.find(c) != -1: 
                            self.publish_func(words) 
                except sr.UnknownValueError:  
                    #rospy.loginfo("Sphinx could not understand audio")  
                    pass
                except sr.RequestError as e:  
                    print("Sphinx error; {0}".format(e))  
            
if __name__ == '__main__':
    cammands = ['stop', 'go forword', 'go left', 'go right', 'go down', 'meet', 'find']
    com = Commander(cammands)
    com.lis_mic()
