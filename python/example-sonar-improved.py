#!/usr/bin/python
# -*- coding: utf-8 -*-


import time
import virtGPIO as GPIO

class Sonar:
  def __init__(self, trigger, echo):
      self.trigPin = trigger
      self.echoPin = echo
      GPIO.output(trigger, GPIO.LOW)  # attempt to pre-set it low before enabling output
      GPIO.setup(trigger, GPIO.OUT)
      GPIO.output(trigger, GPIO.LOW)
      GPIO.setup(echo, GPIO.IN)


  def ping(self):
      GPIO.pulseOut(self.trigPin, GPIO.HIGH, 10)
      GPIO.Serial.timeout = 1.2
      a = GPIO.pulseIn(self.echoPin, GPIO.HIGH, 600, 22000)
      GPIO.Serial.timeout = GPIO.STDTIMEOUT
      if a<18750:    # 250 cm
        return a//70   # result in cm
      else:
        return 0
      # In air, sound travels about 1 cm / 29 uSecs. Wikipedia. Return trip = 58 uSec / cm of range.
      # That's the theory! ie result = a/58.
      # However, my empirical calibration gives formula a/70.   YMMV - correct the scaling for yourself!


sonar1 = Sonar(4,6)   # trig echo

lastDist = 0
while True:

    dist = sonar1.ping()
    ratio = 1.0 # default for div by zero case
    if dist > 0:
      ratio = 1.0 * lastDist // dist
    lastDist = dist
    # lets filter out wildly fluctuating values, also anything over 2 metres
    # also, dist<0, the ERROR return, should be discarded
    if ratio>1.15 or ratio < 0.88 or dist > 200 or dist<0:
      time.sleep(0.1)
      continue
    print (dist)
    time.sleep(1)
