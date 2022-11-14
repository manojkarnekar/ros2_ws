import RPi.GPIO as gpio
import time

gpio.setmode(gpio.BCM)
gpio.setup(12, gpio.OUT)
#gpio.setup(17, gpio.OUT)

try:
   print("Set GPIO High")
   gpio.output(12, gpio.HIGH)
   time.sleep(11)
   gpio.output(12, gpio.LOW)
  # gpio.output(17, gpio.LOW)

except KeyboardInterrupt:
   print("Keyboard Interrupt")

except:
   print("Some Error")

finally:
   print("clean up")
   gpio.cleanup()

