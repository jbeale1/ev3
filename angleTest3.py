#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase

from math import cos
from threading import Thread  # do more than one thing at once
from time import sleep
# ==================================================

def printAngle():
  global angle   # update current heading angle

  i=0
  told = watch.time()/1000 # units of seconds
  while not done:
    #rate = -gy.rate()  # gyro reading
    tnow = watch.time()/1000 # units of seconds
    #dt = tnow - told  # delta-t for this cycle of loop
    #angle += rate*dt  # degrees = (deg/sec) * sec
    mot_angle = mB.angle()  # motor encoder reading
    g_angle = gy.angle()
    t_elapsed = watch.time()/1000 - tstart
    print("%6.3f, %d, %d" % (t_elapsed,mot_angle,g_angle) )
    told = tnow
    sleep(0.01)

# ============================
    
# Initialize motor and timer

done = False         # flag to halt program

watch = StopWatch()
amplitude = 90

mB = Motor(Port.B)  # initialize two large motors
mC = Motor(Port.C)
mB.reset_angle(0)
mC.reset_angle(0)

gy = GyroSensor(Port.S1)
gy.mode='GYRO-RATE' # deliver turn-rate (must integrate for angle)
sleep(1)
gy.mode='GYRO-ANG'  # changing modes causes recalibration
sleep(1)
gy.reset_angle(0)

t = Thread(target=printAngle)
tstart = watch.time() / 1000

t.start()  # start angle-integration routine

# max speed (deg/sec * 10) and accel (deg/s ^2)
#test_motor.set_run_settings(500, 400)

# Run the motor up to 500 deg/s to a target angle of X degrees.
#test_motor.run_target(500, 360)

# Play another beep sound.
# This time with a higher pitch (1000 Hz) and longer duration (500 ms).
brick.sound.beep(100, 10)

# ==========================================

mC.track_target(0)
# mC.stop(Stop.HOLD)

pi = 3.14159265359 # roughly
pi2 = pi * 2.0 # roughly
angB = 0
angC = 0

# wiggle-waggle loop running one wheel, then the other

while True:
  sstart = watch.time()/1000
  mB.reset_angle(0)
  while True:
    seconds = watch.time()/1000 - sstart # time in seconds
    angle_now = (2-(1+cos(seconds))) *amplitude
    mB.track_target(angle_now)
    if (seconds > pi):
      break

  mC.reset_angle(0)
  sstart = watch.time()/1000
  while True:
    seconds = watch.time()/1000 - sstart # time in seconds
    angle_now = (2-(1+cos(seconds))) *amplitude
    mC.track_target(angle_now)
    if (seconds > pi):
      break
