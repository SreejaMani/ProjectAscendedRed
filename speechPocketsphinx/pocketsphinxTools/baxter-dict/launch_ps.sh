#!/bin/bash 
#INDEX=`pacmd list-sources | grep -B1 "name: <alsa_input.usb-Microsoft_Xbox_NUI_Sensor" | grep "index: " | tr -dc '0-9'`
INDEX=`pacmd list-sources | grep -B1 "name: <alsa_input.usb-046d_HD_Pro_Webcam" | grep "index: " | tr -dc '0-9'`
pocketsphinx_continuous -inmic yes -adcdev $INDEX -lm baxter.lm.bin -dict baxter.dict -hmm en-us-baxter -logfn baxter.log
