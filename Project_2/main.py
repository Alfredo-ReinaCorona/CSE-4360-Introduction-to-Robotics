#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


from behavior import *
from project2_pseudocode import *
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.
from random import randint

# Create your objects here.
ev3 = EV3Brick()

# Write your program here.
ev3.speaker.beep()

right_motor = Motor(Port.A)
left_motor = Motor(Port.B)
ult_sonic = UltrasonicSensor(Port.S2)
l_touch_sen = TouchSensor(Port.S4)
r_touch_sen = TouchSensor(Port.S3)
color_sen = ColorSensor(Port.S1)
prop = Motor(Port.C)


# right_motor.run_time(200, 1500, then=Stop.HOLD, wait=False)
# left_motor.run_time(200, 1500, then=Stop.HOLD, wait=False)

left_spd = -50
right_spd = -50
color_reading = color_sen.color()
isWallFollowing = False

isWall = True

while True:
  if color_reading == Color.YELLOW or color_reading == Color.WHITE:
      left_motor.brake()
      right_motor.brake()
      ev3.speaker.say("Found Fire")
      ev3.speaker.beep()
      ev3.speaker.beep()
      prop.run_time(5000,20000,then=Stop.HOLD, wait=True)
      break
  
  wall_follow(isWall, left_motor, right_motor, left_spd, right_spd, ult_sonic, l_touch_sen, r_touch_sen, color_sen, ev3, prop)

