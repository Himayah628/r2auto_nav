import time
import RPi.GPIO as GPIO

# Pin of Input
GPIOpin = 15
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIOpin,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Detect Metal
def detectMetal():
    state = GPIO.input(GPIOpin)
    if not state:
        print("Metal Detected")
    else :
        print("Metal Not Detected")
    
# test module
if __name__ == '__main__':
    while True:
        detectMetal()
        time.sleep(0.2)
