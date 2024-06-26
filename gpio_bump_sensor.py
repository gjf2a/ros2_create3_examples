import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

BUMP_SENSOR_PIN = 17  # Change this to the actual GPIO pin you're using

GPIO.setup(BUMP_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def read_bump_sensor():
    return GPIO.input(BUMP_SENSOR_PIN)

try:
    print("VEX Bump Sensor Reader")
    print("Press Ctrl+C to exit")
    
    while True:
        print(GPIO.input(BUMP_SENSOR_PIN))
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting program")

finally:
    GPIO.cleanup()
