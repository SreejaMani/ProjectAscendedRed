import sys
import os

def playWav(speak):
	os.system('espeak -s 150 -v en+f4 -wtemp.wav "' + speak + '"')
	os.system('./play.sh temp.wav')

playWav(' '.join(sys.argv[1:]))
