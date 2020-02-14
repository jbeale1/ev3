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
  global DTFlag  # signal to write motor drive time in log
  i=0
  told = watch.time()/1000 # units of seconds
  vbat = brick.battery.voltage()/1000.0 # convert millivolts to V
  logfile = open ("log4.csv", "w") # open data log file
  logfile.write("seconds,motor_angle,gyroM,Dg1,Dg2,Dg3\n")
  logfile.write("# EV3 logfile w2.py Vbat = %5.3f\n" % (vbat))
  print("#  Vbat = %5.3f" % (vbat))

  while not done:
    t_elapsed = watch.time()/1000 - tstart
    mot_angle = mB.angle()-mC.angle()  # motor encoder reading
    a1 = gy1.angle()  # first gyro reading
    a2 = gy2.angle()  # second gyro reading
    a3 = gy3.angle()  # third gyro reading
    am = (a1+a2+a3)/3  # average of gyro readings
    i += 1             # ye olde loop counter
    if (i%5 == 0) :
      logfile.write("%6.3f, %d, %d, %d, %d, %d\n" % 
          (t_elapsed,mot_angle,am,am-a1,am-a2,am-a3) )
    if (i%400 == 0):
      vbat = brick.battery.voltage()/1000.0 # convert millivolts to V
      logfile.write("# %6.3f , Vbat = %5.3f\n" % (t_elapsed,vbat))
    if (DTFlag):
      logfile.write("# Drive Time: %5.3f\n" % (driveTime))
      DTFlag = False
    sleep(0.01)  # yield some time to overworked task scheduler

  # Here: (done==True) so finish up and close out
  vbat = brick.battery.voltage()/1000.0 # in millivolts
  logfile.write("# END RUN  Vbat = %5.3f\n" % (vbat))
  logfile.close()  # all done writing to log file
# ==============================================================

done = False         # flag to halt program
DTFlag = False       # flag to report drive time

# =================================
watch = StopWatch()  # timer object

mB = Motor(Port.B)  # initialize two large motors
mC = Motor(Port.C)
robot = DriveBase(mB, mC, 94, 115)  # wheel OD=94mm, wheelbase=115mm

mB.set_run_settings(200, 250) # max speed, max accel
mC.set_run_settings(200, 250) # max speed, max accel

gy1 = GyroSensor(Port.S2)
gy2 = GyroSensor(Port.S3)
gy3 = GyroSensor(Port.S4)

gy1.mode='GYRO-RATE' # deliver turn-rate (must integrate for angle)
gy2.mode='GYRO-RATE' # deliver turn-rate (must integrate for angle)
gy3.mode='GYRO-RATE' # deliver turn-rate (must integrate for angle)
sleep(0.5)
gy1.mode='GYRO-ANG'  # changing modes causes recalibration
gy2.mode='GYRO-ANG'  # changing modes causes recalibration
gy3.mode='GYRO-ANG'  # changing modes causes recalibration
sleep(3)
gy1.reset_angle(0)
gy2.reset_angle(0)
gy3.reset_angle(0)
mB.reset_angle(0)
mC.reset_angle(0)

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
