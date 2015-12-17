#!/bin/bash 
echo "Enter word to be converted: "
read word
echo $word | festival --tts
