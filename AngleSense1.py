#!/usr/bin/env python3

# Turn motor fwd/back through 90 degrees and measure gyro sensor
# Continually integrates gyro turn rate to calculate heading angle
# J.Beale 3-Feb-2020

from ev3dev.ev3 import *
from time import sleep, time
from threading import Thread  # do more than one thing at once
# ==================================================

def printAngle():
  global angle   # update current heading angle

  i=0
  told = time()
  while not done:
    rate = -gy.value()  # gyro reading
    tnow = time()
    dt = tnow - told  # delta-t for this cycle of loop
    angle += rate*dt  # degrees = (deg/sec) * sec
    mot_angle = m.position  # motor encoder reading
    diff = mot_angle - angle
    t_elapsed = time() - tstart
    print("%6.3f, %d, %d, %d" % (t_elapsed,idx,mot_angle,diff) )
    told = tnow

# sleep 0.01: avg. loop time: 15.46 msec =>  64.7 Hz
#   no sleep:  avg. loop time: 4.92 msec => 203.2 Hz
# ==================================================

m = LargeMotor('outB')  # use this motor

# Connect gyro and touch sensors to any sensor ports
gy = GyroSensor() 
ts = TouchSensor()

gy.mode='GYRO-ANG'  # changing modes causes recalibration
gy.mode='GYRO-RATE' # deliver turn-rate (must integrate for angle)

units = gy.units  # reports 'deg' meaning degrees
done = False  # coordinate halting program

t = Thread(target=printAngle)
tstart = time()
t.start()  # start angle-measuring routine
# motor speed -1000...+1000 (units of 0.1 degree per sec)

idx = 0
angle = 0  # call our starting position angle=0
m.reset()  # call the current position 0 degrees
speed = 50 # valid range from -1000 to +1000
turn_angle = 91

while not done:
  idx += 1
  m.run_to_rel_pos(position_sp=turn_angle, speed_sp=speed, stop_action="hold")
  m.wait_while('running') # wait until motor has stopped
  idx += 1
  sleep(0.5)
  idx += 1
  m.run_to_rel_pos(position_sp=-turn_angle, speed_sp=speed, stop_action="hold")
  m.wait_while('running') # wait until motor has stopped
  idx += 1
  sleep(0.5)
  if (idx >= 24):
    idx += 1
    m.run_to_rel_pos(position_sp=0, speed_sp=100, stop_action="coast")
    sleep(1)
    done=True    # signal all loops to end
