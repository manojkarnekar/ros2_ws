import RPi.GPIO as gpio
import time

gpio.setmode(gpio.BCM)
gpio.setup(17, gpio.OUT)

try:
   print("Set GPIO High")
   gpio.output(17, gpio.HIGH)
   time.sleep(5)
   gpio.output(17, gpio.LOW)

except KeyboardInterrupt:
   print("Keyboard Interrupt")

except:
   print("Some Error")

finally:
   print("clean up")
   gpio.cleanup()

