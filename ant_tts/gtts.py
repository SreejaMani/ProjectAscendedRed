#!/usr/bin/env python
#from gtts import gTTS
from gTTS.gtts.tts import gTTS
import os

t = 'hello world'
tts = gTTS(text= t, lang= 'en')
tts.save("/tmp/temp.mp3")
os.system("mpg321 /tmp/temp.mp3")
os.system("rm /tmp/tempt.mp3")
