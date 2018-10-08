from pocketsphinx import *
import pyaudio

pocketsphinx_dir = os.path.dirname(pocketsphinx.__file__)
model_dir = os.path.join(pocketsphinx_dir, 'model')

config = pocketsphinx.Decoder.default_config()
config.set_string('-hmm', os.path.join(model_dir, 'en-us'))
config.set_string('-keyphrase', 'hello')
config.set_string('-dict', os.path.join(model_dir, 'en-us/cmudict-en-us.dict'))
config.set_float('-kws_threshold', 1e-5)

p = pyaudio.PyAudio()

stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=20480)
stream.start_stream()

while True:
    buf = stream.read(1024)
    if buf:
        decoder.process_raw(buff)
    else:
        break;

if decoder.hyp() is not None:
    print("Hotword Detected")
    decoder.end_utt()
    start_speech_recognition()
    decoder.start_utt()
