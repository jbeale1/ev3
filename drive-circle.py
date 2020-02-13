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

# wait until both motor B+C move less than 1 degree in deltaT seconds
def waitBC():
  deltaT = 0.2  # how long to wait between readings (seconds)
  runB = True
  runC = True
  while runB or runC:  # wait until both motors have stopped
    aB1 = mB.angle()
    aC1 = mC.angle()
    sleep(deltaT)
    aB2 = mB.angle()
    aC2 = mC.angle()
    runB = not(aB1 == aB2)
    runC = not(aC1 == aC2)
    # print("MotorB: %d,%d  MotorC: %d,%d" %(aB1,aB2,aC1,aC2))

def printAngle():
  global angle   # update current heading angle
  global DTFlag  # signal to write motor drive time in log
  i=0
  told = watch.time()/1000 # units of seconds
  vbat = brick.battery.voltage()/1000.0 # convert millivolts to V
  logfile.write("# EV3 logfile w2.py Vbat = %5.3f\n" % (vbat))
  print("#  Vbat = %5.3f" % (vbat))

  while not done:
    #rate = gy.speed()  # gyro reading
    tnow = watch.time()/1000 # units of seconds
    #dt = tnow - told  # delta-t for this cycle of loop
    #angle += rate*dt  # degrees = (deg/sec) * sec
    angle = gy.angle()
    mot_angle = mB.angle()-mC.angle()  # motor encoder reading
    # g_angle = gy.angle()
    t_elapsed = watch.time()/1000 - tstart
    a2 = gy2.angle()  # second gyro reading
    i += 1
    if (i%5 == 0) :
      logfile.write("%6.3f, %d, %d, %d, %d\n" % (t_elapsed,mot_angle,angle,a2,angle-a2) )
      #print("%d, %d" % ( mot_angle,angle-a2))
    if (i%400 == 0):
      vbat = brick.battery.voltage()/1000.0 # convert millivolts to V
      print("#  Vbat = %5.3f" % (vbat))
      logfile.write("# %6.3f , Vbat = %5.3f\n" % (t_elapsed,vbat))
    if (DTFlag):
      logfile.write("# Drive Time: %5.3f\n" % (driveTime))
      DTFlag = False
    told = tnow
    sleep(0.01)

  vbat = brick.battery.voltage()/1000.0 # in millivolts
  print("#  Vbat = %5.3f\n" % (vbat))
  logfile.write("# END RUN  Vbat = %5.3f\n" % (vbat))
  logfile.close()  # all done writing to log file

# ============================
# brick.sound.beep(200, 10) # initial start sound

logfile = open ("log4.csv", "w") # open data log file
logfile.write("seconds,motor_angle,gyro_1,gyro_2,diff\n")

done = False         # flag to halt program
DTFlag = False

watch = StopWatch()
amplitude = 90

# =================================
mB = Motor(Port.B)  # initialize two large motors
mC = Motor(Port.C)
robot = DriveBase(mB, mC, 94, 115)  # wheel OD=94mm, wheelbase=115mm

mB.set_run_settings(200, 250) # max speed, max accel
mC.set_run_settings(200, 250) # max speed, max accel

gy = GyroSensor(Port.S3)
gy2 = GyroSensor(Port.S2)

gy.mode='GYRO-RATE' # deliver turn-rate (must integrate for angle)
gy2.mode='GYRO-RATE' # deliver turn-rate (must integrate for angle)
sleep(0.5)
gy.mode='GYRO-ANG'  # changing modes causes recalibration
gy2.mode='GYRO-ANG'  # changing modes causes recalibration
sleep(3)
gy.reset_angle(0)
gy2.reset_angle(0)
mB.reset_angle(0)
mC.reset_angle(0)

angle = 0  # global var holding accumulated turn angle

runB = False # whether Motor B should be running right now
runC = False

t = Thread(target=printAngle)
tstart = watch.time() / 1000

t.start()  # start angle monitor routine
brick.sound.beep(400, 10) # initialization-complete sound

# ==========================================

speed = 100  # mm per second
steer = 360/8 # degrees per second
msec = 8000   # milliseconds

pi = 3.14159265359 # roughly
pi2 = pi * 2.0 # roughly
angB = 0
angC = 0
oc = 0

waitBC()  # make sure motors are not moving

while oc < 4:
  oc += 1
  cycles = 0
  while (cycles < 2):
    cycles += 1
    print("Cycles: %d,%d" % (oc,cycles))
    t1 = watch.time() / 1000
    robot.drive_time(100, 45, 8000)
    t2 = watch.time() / 1000
    driveTime = t2-t1
    print("CW drive time: %5.3f" % (driveTime))
    DTFlag = True
    sleep(1)  # just in case not quite done
    mB.stop(Stop.HOLD)
    mC.stop(Stop.HOLD)
    waitBC()

  while (cycles < 4):
    cycles += 1
    print("Cycles: %d,%d" % (oc,cycles))
    t1 = watch.time() / 1000
    robot.drive_time(100, -45, 8000)
    t2 = watch.time() / 1000
    driveTime = t2-t1
    print("CCW drive time: %5.3f" % (driveTime))
    DTFlag = True
    sleep(1) # just in case not quite done
    mB.stop(Stop.HOLD)
    mC.stop(Stop.HOLD)
    waitBC()

# =============================================
sleep(2)
waitBC()
done = True
sleep(1)
brick.sound.beep(400, 10) # initialization-complete sound
# =======================================================
