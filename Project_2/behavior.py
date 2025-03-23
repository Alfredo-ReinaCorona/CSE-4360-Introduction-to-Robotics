from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import random

def wall_follow(isWall, left_motor, right_motor, left_spd, right_spd, ultraSonic, l_touch, r_touch, color, ev3, prop):
  distance = ultraSonic.distance(silent=False) / 10 
  l_touched = l_touch.pressed()
  r_touched = r_touch.pressed()
  color_sensed = color.color()

  while True:

    #if color_sensor detects BLUE, break loop and exit
    if color_sensed == Color.YELLOW or color_sensed == Color.WHITE:
      left_motor.brake()
      right_motor.brake()
      break

    #if both touch sensors trigger and there's a wall on the left, turn right
    elif(l_touched == True and r_touched == True and distance < 15):
      left_spd = -100
      right_spd = 50
      left_motor.run_time(200,1000, then=Stop.HOLD, wait=False)
      right_motor.run_time(200,1000,then=Stop.HOLD, wait=True) 
      left_motor.run_time(left_spd, 850, then=Stop.HOLD, wait=False)
      right_motor.run_time(right_spd,850, then=Stop.HOLD, wait=True)
      left_motor.run_time(-200, 500, then=Stop.HOLD, wait=False)
      right_motor.run_time(-200, 500, then=Stop.HOLD, wait=True)
      break

    # if both touch sensors trigger and there's no wall on the left, turn left
    elif(l_touched == True and r_touched == True and distance > 15):
      left_spd = 50
      right_spd = -100
      left_motor.run_time(200,1000, then=Stop.HOLD, wait=False)
      right_motor.run_time(200,1000,then=Stop.HOLD, wait=True) 
      left_motor.run_time(left_spd, 850, then=Stop.HOLD, wait=False)
      right_motor.run_time(right_spd, 850, then=Stop.HOLD, wait=True)
      left_motor.run_time(-200, 500, then=Stop.HOLD, wait=False)
      right_motor.run_time(-200, 500, then=Stop.HOLD, wait=True)
      isWall = False
      #print("both sensors touched")
      break

    # if only left bumper triggeres, turn right
    elif(l_touched == True and r_touched == False):
      left_spd = -100
      right_spd = 50
      left_motor.run_time(200,600, then=Stop.HOLD, wait=False)
      right_motor.run_time(200,600,then=Stop.HOLD, wait=True)             
      left_motor.run_time(left_spd, 850, then=Stop.HOLD, wait=False)
      right_motor.run_time(right_spd, 850, then=Stop.HOLD, wait=True)
      left_motor.run_time(-200, 400, then=Stop.HOLD, wait=False)
      right_motor.run_time(-200, 400, then=Stop.HOLD, wait=True)

    # if only right bumper triggers, and there's a wall on the left, turn right.
    elif(l_touched == False and r_touched == True and distance < 18):
      left_spd = -100
      right_spd = 50
      left_motor.run_time(200,600, then=Stop.HOLD, wait=False)
      right_motor.run_time(200,600,then=Stop.HOLD, wait=True)           
      left_motor.run_time(left_spd, 850, then=Stop.HOLD, wait=False)
      right_motor.run_time(right_spd, 850, then=Stop.HOLD, wait=True)
      left_motor.run_time(-200, 400, then=Stop.HOLD, wait=False)
      right_motor.run_time(-200, 400, then=Stop.HOLD, wait=True)

    # if only the right bumper triggers, turn left
    elif(l_touched == False and r_touched == True):
      left_spd = 50
      right_spd = -100
      left_motor.run_time(200,600, then=Stop.HOLD, wait=False)
      right_motor.run_time(200,600,then=Stop.HOLD, wait=True)           
      left_motor.run_time(left_spd, 850, then=Stop.HOLD, wait=False)
      right_motor.run_time(right_spd, 850, then=Stop.HOLD, wait=True)
      left_motor.run_time(-200, 400, then=Stop.HOLD, wait=False)
      right_motor.run_time(-200, 400, then=Stop.HOLD, wait=True)

    # if there free space detected on the left, break loop
    #TODO Add the wanderinf function after it finds freespace
    elif(distance > 30):
      # left_motor.brake()
      # right_motor.brake()

      #Change the run time if needed
      # Will ensure the robot turns left as soon as it exits wall following
      left_motor.run_time(-200, 1200, then=Stop.HOLD, wait=False)
      right_motor.run_time(-200, 1200, then=Stop.HOLD, wait=True)

      wander(isWall, left_motor, right_motor, left_spd, right_spd, ultraSonic, l_touch, r_touch, color, ev3, prop)
      break

    # if drifting too close to the wall, slight turn to the right
    elif(distance < 7):
      left_spd = -100
      right_spd = 50        
      left_motor.run_time(200,150, then=Stop.HOLD, wait=False)
      right_motor.run_time(200,150,then=Stop.HOLD, wait=True)             
      left_motor.run_time(left_spd, 250, then=Stop.HOLD, wait=False)
      right_motor.run_time(right_spd, 250, then=Stop.HOLD, wait=True)
      left_motor.run_time(-200, 500, then=Stop.HOLD, wait=False)
      right_motor.run_time(-200, 500, then=Stop.HOLD, wait=True)

    #if drifing too far away from the wall, slight turn to the left
    elif(distance > 7):
      left_spd = 50
      right_spd = -100
      left_motor.run_time(200,150, then=Stop.HOLD, wait=False)
      right_motor.run_time(200,150,then=Stop.HOLD, wait=True)           
      left_motor.run_time(left_spd, 250, then=Stop.HOLD, wait=False)
      right_motor.run_time(right_spd, 250, then=Stop.HOLD, wait=True)
      left_motor.run_time(-200, 500, then=Stop.HOLD, wait=False)
      right_motor.run_time(-200, 500, then=Stop.HOLD, wait=True)

    # else, drive forward
    else:
      left_spd = -50
      right_spd = -50
      left_motor.run(left_spd)
      right_motor.run(right_spd)

    #read sensors at the end of the loop  
    distance = ultraSonic.distance(silent=False) / 10
    l_touched = l_touch.pressed()
    r_touched = r_touch.pressed()
    color_sensed = color.color()
  
  # we might not need to pass this boolean value into the function call
  isWall = False


def wander(isWall, left_motor, right_motor, left_spd, right_spd, ultraSonic, l_touch, r_touch, color, ev3, prop):
  isWander = True
  

  #FORWARD is NEGATIVE
  while isWander:
    randTime = random.randrange(300,400)
    l_touched = l_touch.pressed()
    r_touched = r_touch.pressed()
    color_sensed = color.color()

    distance = ultraSonic.distance(silent=False) / 10 
    print(distance)

    #Check if the colored paper has been found
    if color_sensed == Color.WHITE or color_sensed == Color.YELLOW  :
      ev3.speaker.say("Found Fire")
      ev3.speaker.beep()
      ev3.speaker.beep()
      left_motor.brake()
      right_motor.brake()
      prop.run_time(5000,20000,then=Stop.HOLD, wait=True)
      exit(0)
    
    #Hits a wall but there is no wall on the left
    elif(l_touched == True and r_touched == True and distance > 15) :
      
      #Back up
      left_motor.run_time(180,1000, then=Stop.HOLD, wait=False) 
      right_motor.run_time(180,1000,then=Stop.HOLD, wait=True)

      #Turn around
      left_motor.run_time(200,2000, then=Stop.HOLD, wait=False) 
      right_motor.run_time(-220,2000,then=Stop.HOLD, wait=True)

      #pick random direction
      if(randTime % 2 == 0):
        left_motor.run_time(200, randTime, then=Stop.HOLD, wait = False)
        right_motor.run_time(-200, randTime, then=Stop.HOLD, wait = True)
      else:
        left_motor.run_time(-200, randTime, then=Stop.HOLD, wait = False)
        right_motor.run_time(200, randTime, then=Stop.HOLD, wait = True)

    #Hits a wall and there is a wall on the left
    elif(l_touched == True and r_touched == True and distance < 15):
      break

    #Hits left bumper and wall is not under the threshold
    #DOUBLE
    elif(l_touched == True and r_touched == False and distance > 15):
      #Back up
      left_motor.run_time(180,1000, then=Stop.HOLD, wait=False) 
      right_motor.run_time(180,1000,then=Stop.HOLD, wait=True)

      #Turn right a little
      left_motor.run_time(-150,500, then=Stop.HOLD, wait=False) 
      right_motor.run_time(150,500,then=Stop.HOLD, wait=True)

      left_motor.run_time(-200,1000, then=Stop.HOLD, wait=False) 
      right_motor.run_time(-200,1000,then=Stop.HOLD, wait=True)
    #Hits right bumper and wall is not under threshold
    elif(l_touched == False and r_touched == True and distance > 15):
      #Back up
      left_motor.run_time(180,1000, then=Stop.HOLD, wait=False) 
      right_motor.run_time(180,1000,then=Stop.HOLD, wait=True)

      #Turn left a little
      left_motor.run_time(150,500, then=Stop.HOLD, wait=False) 
      right_motor.run_time(-150,500,then=Stop.HOLD, wait=True)
      left_motor.run_time(-200,1000, then=Stop.HOLD, wait=False) 
      right_motor.run_time(-200,1000,then=Stop.HOLD, wait=True)
    #Hits right bumper and there is a wall in range
    elif(l_touched == False and r_touched == True and distance < 15):
      break

    #If the robot arbitrarily detects a wall within range
    elif(distance < 20):
      break

    #Hits left bumper and it's not under threshold
    #DOUBLE
    elif( l_touched == True and r_touched == False and distance > 15):
      #Turn left a little
      left_motor.run_time(150,400, then=Stop.HOLD, wait=False) 
      right_motor.run_time(-150,400,then=Stop.HOLD, wait=True)

    # No walls hit or detected, wander
    else:

      #Goes straight, then turns, maybe switch it around

      left_motor.run_time(-180,500, then=Stop.HOLD, wait=False) 
      right_motor.run_time(-180,500,then=Stop.HOLD, wait=True)

      #pick random direction
      if(randTime % 2 == 0):
        left_motor.run_time(200, randTime, then=Stop.HOLD, wait = False)
        right_motor.run_time(-200, randTime, then=Stop.HOLD, wait = True)
      else:
        left_motor.run_time(-200, randTime, then=Stop.HOLD, wait = False)
        right_motor.run_time(200, randTime, then=Stop.HOLD, wait = True)