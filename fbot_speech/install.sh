#!/bin/sh

[ $(which python3-pyaudio) ] || sudo apt install python3-pyaudio


pip install -r ./requirements.txt
