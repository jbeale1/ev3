# Line follower with two sensors, in online EV3 simulator
# https://www.aposteriori.com.sg/Ev3devSim/index.html

# "FLL 2019 - City Shaper" background
# Robot start: X:585 Y:215 Angle:50

# follows upper line VERY slowly; takes 1.5 minutes
# 96.233, 0.1217, 1100, -221, 000,  5.0, 16.5, 3802.5

"""


 Robot Config as shown:
{
  "wheeldiameter": 56,
  "wheelSpacing": 180,
  "back": -120,
  "weight": "medium",
  "sensor1": {
    "x": -20,
    "y": 30
  },
  "sensor2": {
    "x": 20,
    "y": 30
  },
  "ultrasonic": {
    "x": 0,
    "y": 20,
    "angle": 0
  }
}
"""

from ev3dev2.motor import MoveSteering, OUTPUT_B, OUTPUT_C, LargeMotor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor, GyroSensor, UltrasonicSensor
import time

def sign(x):  # would be nice if Python had basic math functions
  if (x > 0):
    return 1
  elif (x == 0):
    return 0
  else:
    return -1

steering_drive = MoveSteering(OUTPUT_B, OUTPUT_C)

mB = LargeMotor(OUTPUT_B) # individual motor B
mC = LargeMotor(OUTPUT_C) #

colorLeft = ColorSensor(INPUT_2)
colorRight = ColorSensor(INPUT_3)
gyro = GyroSensor(INPUT_4)

# ==== KEY TUNING PARAMETERS ==================
GAIN = 0.75  # proportional feedback gain. Higher = more oscillations
smax = 25    # max drive speed = (smax + smin) 
smin = 5    # minimum drive speed
# ------------------------------------------------

cycles = 0            # how many cycles through the main loop
odAvg = 0             # (L+R) average wheel odometer reading
tStart = time.time()  # seconds
tPrevious = 0         # elapsed time last cycle
odAvgLast = 0         # average odometer last cycle
vScaleFac = 0.1       # fudge factor for "actual" motor speed

print("Time, dT, Cyc, deg, deg/s, reqSpeed, realSpeed, dist") # column headings

while (odAvg < 3800): # stop after a certain distance
  tElapsed = time.time() - tStart  # duration in seconds
  dT = tElapsed - tPrevious
  cycles += 1                      # just a loop counter
  angle,rate = gyro.angle_and_rate
  odB = mB.position # cumulative angle on B motor
  odC = mC.position # cumulative angle on C motor
  odAvg = ((odB + odC) / 2)
  dX = odAvg - odAvgLast  # change in odometer from last cycle
  vel = dX/dT * vScaleFac # apparent actual motor speed in simulation
  error = colorLeft.reflected_light_intensity - colorRight.reflected_light_intensity
  if (odAvg < 250):  # ignore anything before start of line
    correction = 0 # just drive straight
    speed = smax   # pedal to the metal, getting out of launch area
  else:
    correction = error * GAIN
    if (abs(correction) > 100):  # max valid correction is +/- 100
      correction = 100 * sign(correction)
    # slow down when correcting and/or turning
    speed = smin + smax*(100-(abs(correction*3) + abs(rate*0.5)))/100
    if (speed < smin):  # but always go at least this fast
      speed = smin
  print("%05.3f, %5.4f, %03d, %03d, %03d, %4.1f, %4.1f, %5.1f" %
        (tElapsed, dT, cycles, angle, rate, speed, vel, odAvg))
  steering_drive.on(correction, speed) # drive with this angle and speed
  tPrevious = tElapsed
  odAvgLast = odAvg
# ==========================================================  
