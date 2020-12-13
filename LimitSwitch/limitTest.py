# Monitor all piTFT buttons and print when they have been pressed
# Quit script when 27 has been pressed
# cec272, hh559
# Lab 1
# 10/1/2020
#
# jfs9 2/9/20  GPIO example python script
#
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)   # Set for broadcom numbering not board numbers...
#                        V need this so that button doesn't 'float'!
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(14, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_UP)

run = True

while run:
    time.sleep(0.2)  # Without sleep, no screen output!
    if ( not GPIO.input(5) ):
        print (" ") 
        print ("Button 5 pressed....")
    if ( not GPIO.input(14) ):
        print (" ") 
        print ("Button 14 pressed....")
    if ( not GPIO.input(15) ):
        print (" ") 
        print ("Button 15 pressed....")

