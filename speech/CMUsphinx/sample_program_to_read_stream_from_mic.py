#!/usr/bin/python

from os import environ, path
import os
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio
import wave
import socket
import time
import random

#pocketsphinx_continuous -adcdev sysdefault -hmm /usr/local/share/pocketsphinx/model/en-us/en-us -lm 9735.lm -dict 9735.dic -samprate 16000 -inmic yes

MODELDIR = "/anaconda3/lib/python3.6/site-packages/pocketsphinx/model"
DATADIR = "/Users/yakun/Documents/18S2/COMP9431RSA/proj2/speech_reg/CMUsphinx/TAR0257"

# Create a decoder with certain model
config = Decoder.default_config()
#config.set_string('-adcdev', 'sysdefault')
#config.set_string('-adcdev', 'plughw:0,0')
config.set_string('-hmm', os.path.join(MODELDIR, 'en-us'))
config.set_string('-lm', os.path.join(DATADIR, '0257.lm'))
#config.set_string('-kws', '/home/perf/Downloads/TrevorWarren/Python/Dev/PocketSpinx_TTS/data/keywords')
config.set_string('-dict', os.path.join(DATADIR, '0257.dic'))
#config.set_string('-samprate', '8000')
#config.set_string('-inmic', 'yes')
decoder = Decoder(config)

p = pyaudio.PyAudio()
#stream = p.open(format=pyaudio.paInt16, channels=2, rate=16000, input=True, frames_per_buffer=1024, input_device_index=2)
#stream = p.open(format=pyaudio.paInt16, channels=1, rate=8000, input=True, frames_per_buffer=1024, input_device_index=0)
#stream = p.open(format=pyaudio.paInt16, channels=1, rate=8000, input=True, frames_per_buffer=1024, input_device_index=0)
stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
stream.start_stream()


decoder.start_utt()
print("Starting to listen")

while True:
    buf = stream.read(1024)
    decoder.process_raw(buf, False, False)
    if decoder.hyp() != None and decoder.hyp().hypstr == 'MOVE FORWARD':
        decoder.end_utt
        print("Detected Move Forward, moving forward and restarting search")
        decoder.start_utt()
    elif decoder.hyp() != None and decoder.hyp().hypstr == 'MOVE BACKWARD':
        decoder.end_utt
        print("Detected Move Backward, moving backward and restarting search")
        decoder.start_utt()
    elif decoder.hyp() != None and decoder.hyp().hypstr == 'MOVE LEFT':
        decoder.end_utt
        print("Detected Move Left, moving left and restarting search")
        decoder.start_utt()
    elif decoder.hyp() != None and decoder.hyp().hypstr == 'MOVE RIGHT':
        decoder.end_utt
        print("Detected Move Right, moving right and restarting search")
        decoder.start_utt()
    elif decoder.hyp() != None and decoder.hyp().hypstr == 'STOP':
        decoder.end_utt
        print("Detected Stop. Stopping and restarting search")
        decoder.start_utt()
    else:
        decoder.end_utt()
        print("Nothing Detected, Restarting search" + str(random.randint(0,100)))
        decoder.start_utt()
print("Am not listening any more")
