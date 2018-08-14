#!/usr/bin/python

import sys, os
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *


modeldir = "../../pocketsphinx/model"
datadir = "../../pocketsphinx/test/data"

keyphrase = "forward"

# Create a decoder with certain model
config = Decoder.default_config()
config.set_string('-hmm', os.path.join(modeldir, 'en-us/en-us'))
config.set_string('-dict', os.path.join(modeldir, 'en-us/cmudict-en-us.dict'))
config.set_string('-keyphrase', keyphrase)
config.set_float('-kws_threshold', 1e+20)

# Open file to read the data
# stream = open(os.path.join(datadir, "goforward.raw"), "rb")

# Alternatively you can read from microphone
import pyaudio

p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16, 
                channels=1, 
                rate=16000, 
                input=True, 
                frames_per_buffer=2048, 
                input_device_index=8
                )
stream.start_stream()

# Process audio chunk by chunk. On keyphrase detected perform action and restart search
decoder = Decoder(config)
print ("Pronunciation for word '"+keyphrase+"' is ", decoder.lookup_word(keyphrase))

decoder.start_utt()
while True:
    buf = stream.read(1024)
    if buf:
         decoder.process_raw(buf, False, False)
    else:
         break
    if decoder.hyp() != None:
        print ([(seg.word, seg.prob, seg.start_frame, seg.end_frame) for seg in decoder.seg()])
        print ("Detected keyphrase, restarting search")
        decoder.end_utt()
        decoder.start_utt()


# import pyaudio
# p = pyaudio.PyAudio()

# for i in range(p.get_device_count()):
#     dev = p.get_device_info_by_index(i)
#     print((i,dev['name'],dev['maxInputChannels']))