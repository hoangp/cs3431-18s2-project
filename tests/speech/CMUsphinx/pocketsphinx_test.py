#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 31 09:15:12 2018

@author: yakun
"""

from pocketsphinx import LiveSpeech
print("Say something!")
for phrase in LiveSpeech(): 
    print("This is what you said:\n")
    print(phrase)