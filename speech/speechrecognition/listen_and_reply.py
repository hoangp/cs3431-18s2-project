# -*- coding: utf-8 -*-

#!/usr/bin/env python

#you need to get your own json from original website
#https://cloud.google.com/speech-to-text/docs/quickstart-client-libraries
import speech_recognition as sr
import requests
import json
import sys
import re


def reg_speech_from_mic(recognizer,microphone):
    """Transcribe speech from recorded from `microphone`.
        
    Returns a dictionary with three keys:
    "success": a boolean indicating whether or not the API request was
                successful
    "error":   `None` if no error occured, otherwise a string containing
                an error message if the API could not be reached or
                speech was unrecognizable
    "transcription": `None` if speech could not be transcribed,
                otherwise a string containing the transcribed text
    """
    # check that recognizer and microphone arguments are appropriate type
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")
    
    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    # obtain audio from the microphone
    with microphone as source:
        # listen for 5 seconds and create the ambient noise energy level
        recognizer.adjust_for_ambient_noise(source)
        print("Say something!")
        audio = recognizer.listen(source)

    response = {
        "success": True,
        "error" : None,
        "transcription" : None
    }
    
    # recognize speech using Sphinx
    try:
        response["transcription"] = recognizer.recognize_google_cloud(audio, credentials_json=credentials_json)
    except sr.UnknownValueError:
        response["error"] = "Could not recoginize what you said"
    except sr.RequestError as e:
        response["success"] = False
        response["error"] = "API Problem"

    return response

def greet_detect(reg,speech):
    matching=re.search(reg, speech)
    if not matching:
        print("hello")
    return matching

def name_detect(reg,speech):
    matching=re.search(reg, speech)
    if not matching:
        print("hello ")
    return matching

if __name__ == "__main__":
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()
    #pattern = "how are you"
    
    with open(r"../google_cloud/service-account-file.json", "r") as f:
        credentials_json = f.read()
    
    response = reg_speech_from_mic(recognizer,microphone)
    print(response)
    if not response["transcription"]:
        print("I didn't catch that. What did you say?\n")
        sys.exit()
    if not response["success"]:
        print("I didn't catch that. What did you say?\n")
        sys.exit()
    if response["error"]:
        print("ERROR: {}".format(response["error"] ))
        sys.exit()

    
    print("You said:{}".format(response["transcription"]))
    speech=response["transcription"].lower()
    reg="^\bhow\sare\syou\b$"

    matching = greet_detect(reg,speech)
    name = name_detect(reg,speech)

    print("Matching:",matching)



