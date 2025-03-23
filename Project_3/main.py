#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from pybricks.iodevices import I2CDevice

import random

# IR Sensor Setup
ir_sensor = I2CDevice(Port.S3, 0x01)
# Motor Setup
ev3 = EV3Brick()
right_motor = Motor(Port.A)
left_motor = Motor(Port.B)

# Other Sensors
color_sen = ColorSensor(Port.S1)
gyro_sen = GyroSensor(Port.S2)
gyro_sen.reset_angle(0)
ultraSonic = UltrasonicSensor(Port.S4)

def ir_direction():
  return int.from_bytes(ir_sensor.read(0x42,1),'little')

def ir_strength():
  return int.from_bytes(ir_sensor.read(0x48,1),'little')

#Ball tracking, should dynamically adjust to find the ball
#once it finds the ball it switches to 'aggressive()' which has the call to the 'charge()' function
def ball_search():
  sweep_count = 0
  while True:
    randTime = random.randrange(300,400)
    color_sensed = color_sen.color()
    rsi = ir_strength()
    direction = ir_direction()
    distance = ultraSonic.distance(silent=False) / 10 

    if direction != 0:
      break 

    #if it hits enemy line and ball is not detected
    elif color_sensed == Color.RED:
      #Back up
      left_motor.run_time(-180, 700, then=Stop.COAST, wait = False)
      right_motor.run_time(-180, 700, then=Stop.COAST, wait = True)

      red_turn()

      #pick random direction
      if(randTime % 2 == 0):
        left_motor.run_time(200, randTime, then=Stop.COAST, wait = False)
        right_motor.run_time(-200, randTime, then=Stop.COAST, wait = True)
      else:
        left_motor.run_time(-200, randTime, then=Stop.COAST, wait = False)
        right_motor.run_time(200, randTime, then=Stop.COAST, wait = True)

    #If a wall is detected while wandering
    elif distance < 15:
      #Back up
      left_motor.run_time(-180,500, then=Stop.COAST, wait=False) 
      right_motor.run_time(-180,500,then=Stop.COAST, wait=True)

      #Turn around
      left_motor.run_time(200,1000, then=Stop.COAST, wait=False) 
      right_motor.run_time(-220,1000,then=Stop.COAST, wait=True)

      #pick random direction
      if(randTime % 2 == 0):
        left_motor.run_time(200, randTime, then=Stop.COAST, wait = False)
        right_motor.run_time(-200, randTime, then=Stop.COAST, wait = True)
      else:
        left_motor.run_time(-200, randTime, then=Stop.COAST, wait = False)
        right_motor.run_time(200, randTime, then=Stop.COAST, wait = True)

    # No ball detected, no wall detected
    else:
      #Goes straight, then turns. range(1000) is just a way to run or a certain amount of time whil being able to detect the ball
      for _ in range(1000):
        if color_sen.color() == Color.RED:
          motor_brakes()
          #Back up
          left_motor.run_time(-180, 700, then=Stop.COAST, wait = False)
          right_motor.run_time(-180, 700, then=Stop.COAST, wait = True)
          red_turn()
          break
        #If color is not detected
        left_motor.run(180)
        right_motor.run(180)

      motor_brakes()

      #BEFORE it picks a random direction, do a "sweep" of the area to try and detect the ball
      if sweep_count == 3:
        ##ev3.speaker.say("Sweep")
        current_angle = gyro_sen.angle()
        #Turn left ~20 degrees
        while gyro_sen.angle() > current_angle - 20:
          if color_sen.color() == Color.RED or ir_direction() != 0:
            motor_brakes()
            break
          left_motor.run(-180)
          right_motor.run(180)
        motor_brakes()
        current_angle = gyro_sen.angle()
        #Turn right 40 degrees to cancel out the left turn and look further
        while gyro_sen.angle() < current_angle + 40:
          if color_sen.color() == Color.RED or ir_direction() != 0:
            motor_brakes()
            break
          left_motor.run(180)
          right_motor.run(-180)
        motor_brakes()
        current_angle = gyro_sen.angle()
        #Turn back to center
        while current_angle < current_angle - 20:
          if color_sen.color() == Color.RED or ir_direction() != 0:
            motor_brakes()
            break
          left_motor.run(180)
          right_motor.run(-180)
        motor_brakes()

        sweep_count = 0

      #pick random direction
      if(randTime % 2 == 0):
        for _ in range(50):
          if color_sen.color() == Color.RED or int.from_bytes(ir_sensor.read(0x42,1),'little') != 0:
            motor_brakes()
            break
          left_motor.run(150)
          right_motor.run(-150)
      else:
        for _ in range(50):
          if color_sen.color() == Color.RED or int.from_bytes(ir_sensor.read(0x42,1),'little') != 0:
            motor_brakes()
            break
          left_motor.run(-150)
          right_motor.run(150)
      sweep_count += 1

def aggressive():
  lft_spd = 75
  rght_spd = 75
  direction = ir_direction()
  rsi = ir_strength()
  distance = ultraSonic.distance(silent=False) / 10 
  while direction != 0:
    # If enemy line detected
    if color_sen.color() == Color.RED:
      motor_brakes()
      break
    # if ball detected to the right and rsi < 15
    elif direction > 5 and rsi < 15:
      motor_brakes()      
      while ir_direction() != 5 and ir_direction() != 0 and color_sen.color() != Color.RED:
        right_motor.run(-rght_spd)
        left_motor.run(lft_spd)
      motor_brakes()
    # if ball detected to the left and rsi < 15
    elif direction < 5 and rsi < 15:
      motor_brakes()    
      while ir_direction() != 5 and ir_direction() != 0 and color_sen.color() != Color.RED:
        right_motor.run(rght_spd)
        left_motor.run(-lft_spd)
      motor_brakes()
    # if ball detected straight ahead, rsi > 15, and facing toward enemy goal
    elif direction == 5 and rsi > 15 and gyro_sen.angle() % 360 >= 315 or gyro_sen.angle() % 360 <= 45 and color_sen.color() != Color.RED:
      # ev3.speaker.say(str(gyro_sen.angle()%360))
      charge()
    elif direction == 5 and rsi < 15:
      while ir_strength() < 15 and ir_direction() != 0:
        left_motor.run(65)
        right_motor.run(65)
      motor_brakes()
    # if ball detected straight ahead, rsi > 20, facing towards our goal
    elif direction == 5 and rsi > 35 and gyro_sen.angle() % 360 < 315 and gyro_sen.angle() % 360 > 180:
      motor_brakes()
      while gyro_sen.angle() % 360 < 335 and ir_direction() != 0:
        left_motor.run(75)
        right_motor.run(-75)
        if ir_strength() < 30:
          aggressive()
      motor_brakes()
      dribble()
    # if ball detected straight ahead, rsi > 30, facing towards enemy goal
    # elif direction == 5 and rsi > 35 and gyro_sen.angle()  % 360 < 270 and gyro_sen.angle() % 360 < 180:
    #   motor_brakes()
    #   while gyro_sen.angle() % 360 < -335 and ir_direction() != 0:
    #     left_motor.run(-75)
    #     right_motor.run(75)
    #   motor_brakes()
    #   dribble()
    # if ball detected straight ahead, rsi > 30, facing towards our goal
    # elif direction == 5 and rsi > 30 and gyro_sen.angle()  % 360 > 90 and gyro_sen.angle() % 360 > 180:
    #   while gyro_sen.angle() % 360 < 335 and ir_direction() != 0:
    #     if color_sen.color() == Color.RED:
    #       motor_brakes()
    #       break
    #     left_motor.run(75)
    #     right_motor.run(-75)
    #   motor_brakes()
    #   dribble()
    # if ball detected straight ahead, rsi > 20, afacing towards our goal
    elif direction == 5 and rsi > 20 and gyro_sen.angle()  % 360 > 45 and gyro_sen.angle() % 360 < 180:
      motor_brakes()
      while gyro_sen.angle() % 360 > 25 and ir_direction() != 0:
        if color_sen.color() == Color.RED:
          break
        left_motor.run(-75)
        right_motor.run(75)
        if ir_strength() < 30:
          aggressive()
      motor_brakes()
      dribble()
    # # If the ball is perpendicular(within error) to the left wall
    # elif direction == 5 and rsi > 20 and (gyro_sen.angle() <= 97 or gyro_sen.angle() >= 83):
    #   motor_brakes()
    #   ev3.speaker.say("in one")
    #   if distance > 15:
    #     ev3.speaker.say("two")
    #     while distance > 15:
    #       distance = ultraSonic.distance(silent=False) / 10 
    #       if color_sen.color() == Color.RED:
    #         break
    #       elif color_sen.color() != Color.RED and distance > 15:
    #         charge()
    #       elif distance < 15:
    #         ev3.speaker.say("Turn to goal")
    #         while gyro_sen.angle() != 0:
    #           left_motor.run(-75)
    #           right_motor.run(75)
    #     motor_brakes()
    #     ev3.speaker.say("exit case 1")
    # #same case but for the right wall
    # elif direction == 5 and rsi > 20 and (gyro_sen.angle() <= -97 or gyro_sen.angle() >= -83):
    #   motor_brakes()
    #   ev3.speaker.say("in two")
    #   if distance > 15:
    #     ev3.speaker.say("two")
    #     while distance > 15:
    #       distance = ultraSonic.distance(silent=False) / 10 
    #       if color_sen.color() == Color.RED:
    #         break
    #       elif color_sen.color() != Color.RED and distance > 15:
    #         charge()
    #       elif distance < 15:
    #         ev3.speaker.say("Turn to goal")
    #         while gyro_sen.angle() != 0:
    #           left_motor.run(75)
    #           right_motor.run(-75)
    #     motor_brakes()
    #     ev3.speaker.say("exit case 2")
    else:
      left_motor.run(lft_spd)
      right_motor.run(rght_spd) 
    direction = ir_direction()
    rsi = ir_strength()
    distance = ultraSonic.distance(silent=False) / 10
    if color_sen.color() == Color.RED or ir_direction() == 0:
      break
  motor_brakes()

# this is an experimental charge function
def charge2():
  rght_spd = 2000
  lft_spd = 2000
  for x in range(500):
    if color_sen.color() == Color.RED:
      motor_brakes()
      left_motor.run_time(-180, 700, then=Stop.COAST, wait=False)
      right_motor.run_time(-180, 700, then=Stop.COAST, wait=True)

      red_turn()
      break
    left_motor.run(lft_spd)
    right_motor.run(rght_spd)
  motor_brakes()

def charge():
  rght_spd = 2000
  lft_spd = 2000
  for x in range(250):
    if color_sen.color() == Color.RED:
      motor_brakes()

      left_motor.run_time(-180, 700, then=Stop.COAST, wait=False)
      right_motor.run_time(-180, 700, then=Stop.COAST, wait=True)

      red_turn()
      break
    left_motor.run(lft_spd)
    right_motor.run(rght_spd)
  motor_brakes()

def dribble():
  rght_spd = 2000
  lft_spd = 2000
  for x in range(200):
    if color_sen.color() == Color.RED:
      motor_brakes()
      return
    right_motor.run(rght_spd)
    left_motor.run(lft_spd)
  motor_brakes()

def motor_brakes():
  left_motor.brake()
  right_motor.brake()

def red_turn():
  #Turn around
  angle = gyro_sen.angle() % 360
  if angle >= 0 and angle < 90:
    left_motor.run_time(200,1000, then=Stop.COAST, wait=False) 
    right_motor.run_time(-220,1000,then=Stop.COAST, wait=True)
  elif angle < -270:
    left_motor.run_time(200,1000, then=Stop.COAST, wait=False) 
    right_motor.run_time(-220,1000,then=Stop.COAST, wait=True)
  elif angle < 0 and angle > -90:
    left_motor.run_time(-200,1000, then=Stop.COAST, wait=False) 
    right_motor.run_time(220,1000,then=Stop.COAST, wait=True)
  elif angle > 270 :
    left_motor.run_time(-200,1000, then=Stop.COAST, wait=False) 
    right_motor.run_time(220,1000,then=Stop.COAST, wait=True)

gyro_sen.reset_angle(0)
#charge2()

while True:
  if color_sen.color() == Color.RED:
    motor_brakes()
    left_motor.run_time(-180, 700, then=Stop.COAST, wait = False)
    right_motor.run_time(-180, 700, then=Stop.COAST, wait = True)

    red_turn()

  direction = ir_direction()
  rsi = ir_strength()
  
  if direction != 0:
    aggressive()
  else: 
    ball_search()