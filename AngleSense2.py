#!/usr/bin/env python3

# Integrate angle from EV3 gyro sensor
# while driving motor forward 90 degrees and back
# several times; collect data to compare angles
# 2020-Feb-04 J.Beale

from ev3dev.ev3 import *
from time import sleep, time
from threading import Thread  # more than one thing at once
import datetime  # time of day
# ==================================================

def calcAngle():
  global angle   # update current heading angle
  global dmax    # max error seen so far
  global dmin    # min error seen so far
  global dmaxTime # when was max error seen
  global dminTime # when was min error seen
  global Tcount  # total cycles through Angle-calc loop

  i=0
  told = time()
  while not done:
    rate = -gy.value()  # gyro reading
    # ra = abs(rate)
    # if (ra > 40):
    #  print(ra)
    tnow = time()
    dt = tnow - told  # delta-t for this cycle of loop
    angle += rate*dt  # degrees = (deg/sec) * sec
    mot_angle = m.position  # motor encoder reading
    diff = mot_angle - angle
    if (diff > dmax):
      t_elapsed = tnow - tstart
      # print("%6.3f, %d, %d, %5.1f" % (t_elapsed,idx,mot_angle,diff) )
      dmax = diff
      dmaxTime = t_elapsed
    elif (diff < dmin):
      t_elapsed = tnow - tstart
      # print("%6.3f, %d, %d, %5.1f" % (t_elapsed,idx,mot_angle,diff) )
      dmin = diff
      dminTime = t_elapsed
    told = tnow
    Tcount += 1

# sleep 0.01: avg. loop time: 15.46 msec =>  64.7 Hz
#   no sleep:  avg. loop time: 4.92 msec => 203.2 Hz
# ==================================================

m = LargeMotor('outB')  # use this motor

# Connect gyro and touch sensors to any sensor ports
gy = GyroSensor()

Alldone = False
outerLoop = 0

print("DateTime,Tdur,Hz,tMax,Max,tMin,Min,finalAngle") # csv header

# ================================
while not Alldone:
  outerLoop += 1
  if (outerLoop >= 10):
    Alldone = True

  gy.mode='GYRO-ANG'  # changing modes causes recalibration
  gy.mode='GYRO-RATE' # deliver turn-rate (must integrate for angle)

  done = False  # coordinate halting program

  dmax = 0  # maximum error between calculated gyro angle and motor angle
  dmin = 0
  dmaxTime = 0
  dminTime = 0
  Tcount = 0 # number of cycles through angle-integrate loop

  tstart = time() # start time of this run

  t = Thread(target=calcAngle) # set up a thread
  t.start()  # start angle-measuring routine on separate thread

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
    if (idx >= 12):
      idx += 1
      finalAngle = angle  # save ending position of sensor
      done=True    # signal all loops to end
      tElapsed = time() - tstart
      sleep(0.2)
      m.run_to_rel_pos(position_sp=0, speed_sp=100, stop_action="coast")
      sps = Tcount / tElapsed
      print(datetime.datetime.now(),end=",")
      print("%5.3f,%5.1f" % (tElapsed, sps),end="," )
      print("%5.2f,%5.1f,%5.2f,%5.1f,%5.2f" %
           (dmaxTime,dmax,dminTime,dmin,finalAngle))
      sleep(4)  # wait before end of this cycle
