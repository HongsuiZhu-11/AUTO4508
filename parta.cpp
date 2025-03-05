extern "C" {
    #include "eyebot.h"
}

#define SAFE_DISTANCE 150   // 机器人与墙保持的安全距离 (cm)
#define FORWARD_SPEED 50   // 直行速度
#define TURN_SPEED 30      // 旋转速度
#define STEP_DISTANCE 50   // 割草机模式的单次前进步长 (cm)
#define ROW_SPACING 200     // 割草机模式的行间距 (cm)

// **传感器编号（根据你的测试结果）**
#define FRONT_SENSOR 1  // **前方传感器**
#define LEFT_SENSOR 2   // **左侧传感器**
#define RIGHT_SENSOR 3  // **右侧传感器**

// **函数声明**
void alignToWall();   // **自动调整角度，使机器人正对墙**
void driveToWall();   // **前进直到距离墙 15cm**
void turnRight();     // **向右转，使墙在左侧**
void driveToCorner(); // **沿墙前进，直到检测到墙角**
void lawnmowerPattern(); // **执行割草机模式**

int main()
{
    LCDClear();
    LCDPrintf("Starting Robot Navigation...\n");

    driveToWall();       // **第二步：前进到墙前 15cm**
    turnRight();         // **第三步：右转，使墙在左侧**
    driveToCorner();     // **第四步：沿着墙前进，直到检测到墙角**
    turnRight();         // **第五步：在墙角处再次右转**
    lawnmowerPattern();  // **第六步：执行割草机模式**

    LCDPrintf("Lawn Mower Pattern Complete. Stopping.\n");
    return 0;
}

// **第二步：向前行驶，直到检测到前方墙壁**
void driveToWall()
{
    LCDPrintf("Moving towards wall...\n");

    while (1)  // **循环直到 `moveDistance == 0`**
    {
        int frontDist = PSDGet(FRONT_SENSOR);  // **获取前方距离**
        int moveDistance = frontDist - SAFE_DISTANCE;  // **计算需要移动的距离**

        if (moveDistance == 0) break;  // **如果已经在 15cm 处，退出循环**

        VWStraight(moveDistance, (moveDistance > 0) ? FORWARD_SPEED : -30);
        VWWait();
    }

    LCDPrintf("Reached the wall at safe distance.\n");
}


void turnRight()
{
    LCDPrintf("Turning Right...\n");

    VWTurn(-90, TURN_SPEED);  
    VWWait();

    LCDPrintf("Turn Completed.\n");
}

// **第四步：沿着墙前进，直到检测到墙角**
void driveToCorner()
{
    LCDPrintf("Going to the corner...\n");

    while (1)  // **循环直到 `moveDistance == 0`**
    {
        int frontDist = PSDGet(FRONT_SENSOR);  // **获取前方距离**
        int moveDistance = frontDist - SAFE_DISTANCE;  // **计算需要移动的距离**

        if (moveDistance <= 2) break;  // **如果已经在 15cm 处，退出循环**

        VWStraight(moveDistance, (moveDistance > 0) ? FORWARD_SPEED : -30);
        VWWait();
    }
}

// **第五步：右转，使机器人朝向空地**
// **这个功能已经由 turnRight() 实现，因此直接调用 turnRight()**

void lawnmowerPattern()
{
    int frontDist;
    int step = 0;  // 记录行数
    int finalMove = ROW_SPACING;  // **默认行间距**
    
    while (1)
    {
        frontDist = PSDGet(FRONT_SENSOR);  // **前方墙壁距离**

        // **如果检测到墙，调整方向**
        if (frontDist < SAFE_DISTANCE)
        {
            VWSetSpeed(0, 0);
            VWWait();

            // **执行第一次转向**
            if (step % 2 == 0)
                VWTurn(-90, TURN_SPEED);  // **右转**
            else
                VWTurn(90, TURN_SPEED); // **左转**

            VWWait();

            // **在第一次转向后，检查前方剩余空间**
            int nextRowSpace = PSDGet(FRONT_SENSOR) - SAFE_DISTANCE;
            if (nextRowSpace <= ROW_SPACING)
            {
                finalMove = nextRowSpace;  // **更新行间距**
                LCDPrintf("Final move detected: Moving only %d cm\n", finalMove);
            }

            // **执行最后一次移动**
            VWStraight(finalMove, FORWARD_SPEED);
            VWWait();

            // **再次转向**
            if (step % 2 == 0)
                VWTurn(90, TURN_SPEED);
            else
                VWTurn(-90, TURN_SPEED);

            VWWait();

            // **如果 `finalMove` 过小，结束程序**
            if (finalMove <= SAFE_DISTANCE)
            {
                LCDPrintf("Final Row Reached. Stopping.\n");
                break;
            }

            step++;  // **更新行数**
        }

        // **继续前进**
        VWStraight(STEP_DISTANCE, FORWARD_SPEED);
        VWWait();

        // **防止无限循环**
        if (step > 10) break;
    }

    LCDPrintf("Lawnmower Pattern Completed.\n");
}

