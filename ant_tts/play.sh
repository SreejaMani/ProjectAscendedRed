#!/bin/bash
CARD=$(aplay -l | grep "card [0-9]: Pro \[SB X-Fi Surround 5.1 Pro\], device [0-9]: USB Audio \[USB Audio\]$" | cut -c 6)
AUDIODEV=hw:$CARD play $1
