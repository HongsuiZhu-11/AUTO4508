from eye import *
from math import *
# Predefined Variable
STEP = 600
SAFE = 400


def lidarPrint(lidar):
    LCDClear()
    LCDMenu('---', '---', '---', 'END')
    for i in range(len(lidar)):
        LCDLine(i, 250 - lidar[i] // 10, i, 250, BLUE)
    LCDLine(180, 0, 180, 250, RED)
    LCDLine(90, 0, 90, 250, WHITE)
    LCDLine(270, 0, 270, 250, GREEN)


def dist(point1, point2):
    """Calculate the Euclidean distance between two points."""
    return ((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2) ** 0.5


def angleDiff(goalX, goalY):
    currentX, currentY, currentTheta = VWGetPosition()
    goalTheta = degrees(atan2(goalY - currentY, goalX - currentX))
    currentTheta = currentTheta % 360
    goalTheta = goalTheta % 360
    turnTheta = int(goalTheta - currentTheta)
    if abs(turnTheta) > 2:
        if turnTheta > 180:
            turnTheta -= 360
        elif turnTheta < -180:
            turnTheta += 360
    return currentX, currentY, turnTheta


def DistBug(goalX=3000, goalY=3000):
    LCDClear()
    Status = ["Forward", "Encounter", "Orbit", "Escape"]
    idx = 0
    approaching = True
    while approaching and KEYRead() != KEY4:
        if Status[idx] == "Forward":
            # first angle adjust
            """currentX, currentY, currentTheta = VWGetPosition()
            goalTheta = degrees(atan2(goalY - currentY,goalX - currentX))
            currentTheta = currentTheta % 360
            goalTheta = goalTheta % 360
            turnTheta = int(goalTheta - currentTheta)
            if abs(turnTheta) > 2:
                if turnTheta > 180:
                    turnTheta -= 360
                elif turnTheta < -180:
                    turnTheta += 360
            """
            currentX, currentY, turnTheta = angleDiff(goalX, goalY)
            VWTurn(turnTheta, 50)
            VWWait()
            distance = dist((currentX, currentY), (goalX, goalY))
            print(f"distance: {distance}")
            if distance <= PSDGet(PSD_FRONT):
                while distance > 50 and KEYRead() != KEY4:
                    lidar = LIDARGet()
                    lidarPrint(lidar)
                    frontLeft = 180 - 10
                    front = 180
                    frontRight = 180 + 10
                    currentX, currentY, turnTheta = angleDiff(goalX, goalY)
                    distance = dist((currentX, currentY), (goalX, goalY))
                    print(currentX, currentY, turnTheta, distance)
                    if lidar[frontLeft] > 100 and lidar[frontRight] > 100:
                        if abs(turnTheta) > 1:
                            VWSetSpeed(120, turnTheta)
                            OSWait(5)
                        else:
                            VWSetSpeed(120, 0)
                            OSWait(5)
                    elif lidar[frontLeft] <= 100:
                        VWCurve(50, -20, 100)
                        VWWait()
                    elif lidar[frontRight] <= 100:
                        VWCurve(50, 20, 100)
                        VWWait()
                    else:
                        break
                currentX, currentY, turnTheta = angleDiff(goalX, goalY)
                distance = dist((currentX, currentY), (goalX, goalY))
                if distance <= 50:
                    VWSetSpeed(0, 0)
                    print("Goal reached.")
                    return True
            moving = True
            while moving and KEYRead() != KEY4:
                lidar = LIDARGet()
                lidarPrint(lidar)
                currentX, currentY, turnTheta = angleDiff(goalX, goalY) # get current position
                distance = dist((currentX, currentY), (goalX, goalY))
                if lidar[180] > SAFE and lidar[150] > 350 and lidar[210] > 350:
                    if abs(turnTheta) > 1:
                        VWSetSpeed(120, turnTheta)
                        OSWait(25)
                    else:
                        VWSetSpeed(120, 0)
                        OSWait(25)
                else:
                    VWSetSpeed(0, 0)
                    OSWait(25)
                    idx = 1
                    moving = False
        elif Status[idx] == "Encounter":
            print("Turning Stage.")
            turning = True
            while turning and KEYRead() != KEY4:
                lidar = LIDARGet()
                lidarPrint(lidar)
                min_distance, min_index = min((value, index) for index, value in enumerate(lidar))
                degDiff = 270 - min_index
                if abs(degDiff) > 5:
                    VWTurn(degDiff, 50)
                    VWWait()
                else:
                    turning = False
                    idx = 2
        elif Status[idx] == "Orbit":
            print("Orbiting Stage.")
            orbiting = True
            away = 0
            currX, currY, currTheta = VWGetPosition()
            coordCheck = (currX, currY)
            minDist = dist(coordCheck, (goalX, goalY))
            while orbiting and KEYRead() != KEY4:
                lidar = LIDARGet()
                lidarPrint(lidar)
                # Define the target range around 270 degrees without wrap-around
                start_degree = 270 - 60
                end_degree = 270 + 60
                # Slice the data to only include the specified range
                range_data = lidar[start_degree:end_degree]
                # Find the minimum distance in the specified range and its index within that slice
                min_distance = min(range_data)
                # Find the first index of this minimum distance within the sliced data
                min_index_in_slice = range_data.index(min_distance)
                # Adjust the index to match the original list's indexing
                min_index = start_degree + min_index_in_slice

                # orbiting command
                VWCurve(40, int(5 * (270 - min_index) + 0.5 * (SAFE - min_distance)), 100)
                VWWait()
                lidar = LIDARGet() # updated lidar
                away += 1
                currX, currY, currTheta = angleDiff(goalX, goalY)
                if away > 10 and dist(coordCheck, (currX, currY)) < 50:
                    VWSetSpeed(0, 0)
                    return False
                distance = dist((currX, currY), (goalX, goalY))
                minDist = min(minDist, distance)
                # free space calc
                lidarDegree = 180 - currTheta
                print(f"lidarDegree: {lidarDegree}")
                freeSpace = lidar[lidarDegree]
                print(f"freeSpace: {freeSpace}, distance: {distance}, minDist: {minDist}")
                if (distance - freeSpace) <= (minDist - STEP):
                    orbiting = False
                    VWSetSpeed(0, 0)
                    idx = 3
        elif Status[idx] == "Escape":
            print("Enter Escape status")
            currX, currY, currTheta = angleDiff(goalX, goalY)
            if PSDGet(PSD_FRONT) > 300:
                VWStraight(300, 120)
                VWWait()
            else:
                VWStraight(int(PSDGet(PSD_FRONT) * 0.5), 120)
                VWWait()
            currX, currY, currTheta = angleDiff(goalX, goalY)
            VWTurn(currTheta, 50)
            VWWait()
            idx = 0


def init(x=200, y=200, theta=0):
    # Cast x, y, and theta to integers if the underlying C function expects integers
    VWSetPosition(int(x), int(y), int(theta))
    OSWait(500)
    VWSetPosition(0, 0, 0)
    VWSetSpeed(0, 0)


if __name__ == '__main__':
    LCDMenu('Start', 'Insert', '---', 'End')
    LCDSetPrintf(18, 25, "Click button to start/end.")
    # initiating the bot position
    init(x=500, y=500, theta=0)
    while True:
        key = KEYWait(ANYKEY)
        if key == KEY1:
            LCDClear()
            LCDMenu('Start', '---', '---', 'End')
            DistBug()
            break
        elif key == KEY2:
            inserting = True
            while inserting:
                try:
                    LCDClear()
                    LCDSetPrintf(18, 25, "Insert coordinate from terminal")
                    LCDMenu('---', 'Insert', '---', 'End')
                    coordinate = input("insert goal coordinate e.g.: 1800, 1800").split(",")
                    x = int(coordinate[0].strip())
                    y = int(coordinate[1].strip())
                    inserting = False
                    DistBug(x, y)
                except Exception as e:
                    LCDClear()
                    LCDSetPrintf(18, 25, "Wrong value. Please follow the format: 1800, 1800")
                    LCDMenu('---', 'Insert', '---', 'End')
            break
        elif key == KEY4:
            LCDClear()
            LCDSetPrintf(18, 35, "Good Bye!")
            LCDMenu('Start', '---', '---', 'End')
            OSWait(1000)
            break
