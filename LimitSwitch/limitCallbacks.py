import time
import RPi.GPIO as GPIO

# Limit switch GPIO pins
LS_1 = 5
LS_2 = 14
LS_3 = 15

GPIO.setmode(GPIO.BCM)  
GPIO.setup(LS_1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LS_2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LS_3, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def LIMIT_SWITCH_1(channel):
    print('reset 1')

def LIMIT_SWITCH_2(channel):
    print('reset 2')

def LIMIT_SWITCH_3(channel):
    print('reset 3')


GPIO.add_event_detect(LS_1,GPIO.RISING,callback=LIMIT_SWITCH_1,bouncetime=300)
GPIO.add_event_detect(LS_2,GPIO.RISING,callback=LIMIT_SWITCH_2,bouncetime=300)
GPIO.add_event_detect(LS_3,GPIO.RISING,callback=LIMIT_SWITCH_3,bouncetime=300)

while True:
    time.sleep(.2)
