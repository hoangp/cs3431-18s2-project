import sys,os
import pocketsphinx as ps
import sphinxbase


def decodeSpeech(hmmd,lmdir,dictp,wavfile):
    speechRec = ps.Decoder(hmm = hmmd, lm = lmdir, dict = dictp)
    wavFile = file(wavfile,'rb')
    wavFile.seek(44)
    speechRec.decode_raw(wavFile)
    result = speechRec.get_hyp()
    return result[0]


if __name__ == "__main__":
    hmdir = '/usr/share/pocketsphinx/model/hmm/wsj1'
    lmd   = '/usr/share/pocketsphinx/model/lm/wsj/wlist5o.3e-7.vp.tg.lm.DMP'
    dictd = '/usr/share/pocketsphinx/model/lm/wsj/wlist5o.dic'
    wavfile = "hello-2.wav"
    recognised = decodeSpeech(hmdir,lmd,dictd,wavfile)
