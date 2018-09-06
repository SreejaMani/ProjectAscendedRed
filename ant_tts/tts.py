import os

def playWav(speak):
	os.system('./tts.sh ' + speak)
