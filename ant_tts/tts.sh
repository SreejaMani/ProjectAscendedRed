#!/bin/bash
CARD=$(aplay -l | grep "card [0-9]: Pro \[SB X-Fi Surround 5.1 Pro\], device [0-9]: USB Audio \[USB Audio\]$" | cut -c 6)
SPEAK="'$*'"
espeak -s 150 -v en+f4 "$SPEAK" --stdout | aplay -c 1 -t wav -r 22050 --device=plughw:$CARD
