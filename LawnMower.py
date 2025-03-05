#!/usr/bin/env python3
from eye import *

WALLDIST = 150  # Distance from wall to stop at
LAP_WALL_RATIO = 2  # Ratio of the wall distance to the lap distance

DRIVE_SPEED = 100  # Speed of the robot
TURN_SPEED = 30  # Speed of the robot when turning
CORRECTION_TURN_INTENSITY = 5  # Angle of the turn correction
CORRECTION_TURN_SPEED = 15  # Angle speed of correction turns

LEFT = 1
NILL = 0
RIGHT = -1

def faceWall():
  """Makes robot face the nearest wall"""
  n = PSDGet(PSD_FRONT)
  VWTurn(360, TURN_SPEED*2)
  
  while(not VWDone()):
    dist = PSDGet(PSD_FRONT)
    if dist < n:
      n = dist
  while(dist > n):
    VWTurn(-1, TURN_SPEED)
    dist = PSDGet(PSD_FRONT)
  VWTurn(0, 15)
  

def drivetoWall(side_buffer_dist=150, side=NILL, distance=PSDGet(PSD_FRONT)):
  """frives forward until a wall is reached"""
  print(f"Driving: {distance}mm to wall")
  VWStraight(distance, DRIVE_SPEED)
  while(not VWDone()):
    front_dist = PSDGet(PSD_FRONT)
    right_dist = PSDGet(PSD_RIGHT)
    left_dist = PSDGet(PSD_LEFT)

    if front_dist <= int(WALLDIST):
      VWStraight(0, DRIVE_SPEED)
      VWTurn(0, TURN_SPEED)
      break
    
    # Given Right is the control distance, if the robot is not at the appropriate distance it will correct
    if right_dist != side_buffer_dist and side == RIGHT:
      leftorright = RIGHT if right_dist > side_buffer_dist else LEFT
      VWTurn(leftorright*CORRECTION_TURN_INTENSITY, CORRECTION_TURN_SPEED)
      VWStraight(distance, DRIVE_SPEED)
# abs(left_dist - side_buffer_dist) < 10
    # Given Left is the control distance, if the robot is not at the appropriate distance it will correct
    elif left_dist != side_buffer_dist and side == LEFT:
      leftorright = LEFT if left_dist > side_buffer_dist else RIGHT
      VWTurn(leftorright*CORRECTION_TURN_INTENSITY, CORRECTION_TURN_SPEED)
      VWStraight(distance, DRIVE_SPEED)
      

    


def gotoStart():
  """Navigates to the beginning point"""
  # Finds he nearest wall and drives to it
  faceWall()
  drivetoWall()
  
  # Turns the robot so the wall is to it's left and drives
  # till it reaches the corner
  VWTurn(RIGHT*90, TURN_SPEED)
  VWWait()
  drivetoWall(WALLDIST, LEFT)
  VWTurn(RIGHT*90, TURN_SPEED)
  VWWait()
  VWSetPosition(0, 0, 0)
  print("Start reached")

def checkTurn():
  difference = VWGetPosition()[2] % 90 != 0
  if difference > 45:
    difference = difference - 90
  return difference
    
def correctTurn():
  difference = checkTurn()
  while difference != 0:
    VWTurn(difference, TURN_SPEED)
    difference = checkTurn()
  VWSetSpeed(0,0)
  VWWait()

def doLaps():
  lap_no = 1
  finished_on_side = NILL
  while(finished_on_side == NILL):
    # From a starting position facing the direction of the first right lap, the robot will drive to the far
    
    # Right side following lap
    print(f"Starting left following lap numbered: {lap_no}")
    print(f"With wall spacing of: {WALLDIST*((LAP_WALL_RATIO*lap_no) - 1)}")
    drivetoWall(WALLDIST*((LAP_WALL_RATIO*lap_no) - 1), LEFT)
    VWSetSpeed(0,0)
    VWTurn(RIGHT*90, TURN_SPEED)
    
    VWWait()
    # Given the robot is within the final lap distance it will do a shortened lap width to the end to cover the whole surface area.
    if(PSDGet(PSD_FRONT) < 2*WALLDIST):
      drivetoWall(WALLDIST, LEFT)
      finished_on_side = LEFT
    else:
      # The robot is not at the end and should continue as normal
      VWStraight(WALLDIST*LAP_WALL_RATIO, DRIVE_SPEED)
      VWWait()
    VWTurn(RIGHT*90, TURN_SPEED)
    VWWait()

    # Right side following lap
    lap_no += 1
    print(f"Starting right following lap numbered: {lap_no}")
    print(f"With wall spacing of: {WALLDIST*((LAP_WALL_RATIO*lap_no) - 1)}")
    drivetoWall(WALLDIST*((LAP_WALL_RATIO*lap_no) - 1), RIGHT)
    VWSetSpeed(0,0)
    VWTurn(LEFT*90, TURN_SPEED)
    VWWait()
    # It has completed the lap length and is now facing the far wall, before it moves forward it 
    # should check that the wall isn't too close ie. final lap.
    
    # Given the robot is within the final lap distance it will do a shortened lap width to the end to cover the whole surface area.
    if(PSDGet(PSD_FRONT) < 2*WALLDIST):
      drivetoWall(WALLDIST, RIGHT)
      finished_on_side = RIGHT
    else:
      VWStraight(WALLDIST*LAP_WALL_RATIO, DRIVE_SPEED)
      VWWait()
    VWTurn(LEFT*90, TURN_SPEED)
    VWWait()
    lap_no += 1

  # The robot has finished the laps and is now driving to complete the final lap. The finished_on_side variable indicates which side the robot finished on.
  # and hence which sensor to use for the penultimate lap.
  print("Final lap")
  drivetoWall(WALLDIST, finished_on_side)


if __name__ == "__main__":
  # Main script runner
  gotoStart()
  doLaps()
  print("Done")
  