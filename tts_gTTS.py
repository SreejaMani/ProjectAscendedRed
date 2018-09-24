#!/usr/bin/env python
from gtts import gTTS
import os

t = 'hello world'
tts = gTTS(text= t, lang= 'en')
tts.save("hello.mp3")
os.system("mpg321 hello.mp3")
