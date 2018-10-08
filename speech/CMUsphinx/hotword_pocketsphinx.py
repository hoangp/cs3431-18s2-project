from pocketsphinx import *
import pyaudio

model_dir = os.path.join(os.path.dirname(pocketsphinx.__file__), 'model')

config = pocketsphinx.Decoder.default_config()
config.set_string('-hmm', os.path.join(model_dir, 'en-us'))
config.set_string('-keyphrase', 'hello')
config.set_string('-dict', os.path.join(model_dir, 'cmudict-en-us.dict'))
config.set_float('-kws_threshold', 1e-5)
decoder = Decoder(config)

p = pyaudio.PyAudio()

stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
stream.start_stream()

decoder.start_utt()
while True:
    buff = stream.read(1024)
    if buff:
        decoder.process_raw(buff,False,False)
    else:
        break

    if decoder.hyp() is not None and decoder.hyp().hypstr == 'hello':
        decoder.end_utt()
        print("Hotword Detected")
        decoder.start_utt()
