 # -*- coding: utf-8 -*-

#!/usr/bin/env python

import speech_recognition as sr
import requests
import json

#you need to get your own json from original website
#https://cloud.google.com/speech-to-text/docs/quickstart-client-libraries
with open(r"google_cloud/service-account-file.json", "r") as f:
    credentials_json = f.read()
# obtain audio from the microphone  
r = sr.Recognizer()  
with sr.Microphone() as source:  
   print("Please wait. Calibrating microphone...")  
   # listen for 5 seconds and create the ambient noise energy level  
   r.adjust_for_ambient_noise(source, duration=5)  
   print("Say something!")  
   audio = r.listen(source)  
   
# recognize speech using Sphinx  
try:
    print("Google Speech Recognition thinks you said " + r.recognize_google_cloud(audio, credentials_json=credentials_json))
except sr.UnknownValueError:  
    print("Google Speech Recognition could not understand audio")
except sr.RequestError as e:  
    print("Google Speech Recognition error; {0}".format(e))  
