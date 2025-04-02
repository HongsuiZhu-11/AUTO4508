# AUTO4508 Project Semester 1 2025 
# Team 10


## Building the Dockerfile

### If the image exeists:
enter the docker container with `$ docker compose up`

### Otherwise:
`docker build -t [image name]`
Note: image name for project should be `first` 
<br>
Then, do the compose step.


## Once the image is built:
`$ setup.sh` and `setaria.sh`

## Useful Resources:

[AUTO4508 Project Outline](https://roblab.org/courses/mobrob/project/general/AUT4508%20-%20Group%20Project%202025%20-%20V1_0.pdf)

[ARIA Developer's Reference Manual](https://www.cassinis.it/Siti%20ex%20Uni/ARL/docs/documentation/Aria%20documentation/Current/Aria/docs/main.html)

## Notes About Aria Development:
### Documentation and Coding Convention
ARIA follows the following coding conventions:
<ul>
<li>Class names begin with "Ar" and are in mixed case.
<li>Enums and other constants either begin with a capital letter or are all in caps.
<li>Avoid preprocessor definitions whenever possible (instead using enumerations or inline methods)
<li>Member variables in classes are prefixed with 'my'.
<li>Static variables in classes are prefixed with 'our'.
<li>Member function names start with a lower case.
<li>Capitalize each word except the first one in a variable or method name; likeThisForExample
<li>All classes may be used in a multi-threaded program, either by being inherently threadsafe, or (more typically) by providing an API for protecting it by locking mutexes. See class <li>documentation for notes on access from multiple threads.
</ul>

### Asynchronous Task Class

<details><summary>[Click to expand.]</summary>
<p>
ARIA provides the ArASyncTask which can be subclassed to implement a long-running thread and its state as an object. As opposed to robot-syncronized tasks, asynchronous tasks run in seperate threads. Like ArMutex, this class wraps the operating system's threading calls in a cross-platform way. Typically, an ArASyncTask will reperesent a thread that runs in a loop for the entire program.

To use ArASyncTask, derive a class from ArASyncTask and override the ArASyncTask::runThread() function. This function is automatically called within the new thread when that new thread gets created. To create and start the thread, call ArASyncTask::create(). When the ArASyncTask::runThread() function exits, the thread will exit and be destroyed. Seperate threads can request that the task exit by calling ArASyncTask::stopRunning(), and within the thread, you can check for this request with ArASyncTask::getRunningWithLock().

This class is mainly a convenience wrapper around ArThread so that you can easily create your own object that encapsulates the concept of a thread.
</p>
</details>

### Robot Communication (Important)

<details><summary>[Click to expand.]</summary>
<p>
One of the most important functions of ARIA, and one of the first and things that your application program must do, is to establish the connection between an ArRobot object instance and the robot platform operating system (firmware).

In addition to the mobile robot itself, some accessories, such as the sonar, the Pioneer Gripper, PTZ cameras, Pioneer Arm, compass, and others, are internally connected to the robot microcontroller's AUX or digital I/O lines, and use the robot connection as well (therefore the interface classes for these objects require a reference to an ArRobot object, which must be connected for the devices to work). Other accessories, such as the SICK laser, video capture cards, etc. are connected directly to the onboard computer.

There are several ways to connect a computer running ARIA to the robot's microcontroller or to a simulator. This figure provides a schematic overview of the many ARIA-robot communication options. Consult your robot Operations Manual for more information about computer-robot hardware setup and communications.
</p>
</details>

### ArGPS
<details><summary>[Click to expand.]</summary>
<p>
ArGPS provides access to data received from a Global Positioning System device. Subclasses implement special actions required for specific devices, such as ArNovatelGPS for the NovAtel G2 and similar devices and ArTrimbleGPS for the Trimble GPS. Use ArGPSConnector to create the appropriate ArGPS device based on robot and program configuration parameters.
</p>
</details>


### ArMaps
<details><summary>[Click to expand.]</summary>
<p>
In mobile robot applications, you will often need to store a map of the robot's environment to use in navigation, localization, etc. ARIA provides the ArMap class for reading map data from a file, obtaining and modifying its contents in your application, and writing it back to file. An ArMap contains data about the sensed/sensable environment (walls, obstacles, etc.), and human-provided objects such as goal points.
</p>
</details>

### Motion Command Functions
<details><summary>[Click to expand.]</summary>
<p>
At a level just above ArRobot's Direct Commands are the Motion Command Functions. These are explicit simple movement commands sent by ArRobot's state reflection task. For example, ArRobot::setVel() to set the translational velocity, ArRobot::setRotVel to set rotational velocity, ArRobot::setVel2() to or set each wheel speeds separately, ArRobot::setHeading() to set a global heading angle to turn to, ArRobot::move() to drive a given distance, or ArRobot::stop() to stop all motion. ArRobot also provides methods for setting speed limits beyond the limits set in the firmware configuration. These motion functions work at part of with State Reflection, and ArRobot may resend commands each cycle to try to achieve the desired state.

Be aware that a Direct or a Motion Command may conflict with controls from Actions or other upper-level processes and lead to unexpected consequences. Use ArRobot::clearDirectMotion() to cancel the overriding effect of a previously set Motion Command so that your Action is able to regain control the robot. Or limit the time a Motion Command prevents other motion actions with ArRobot::setDirectMotionPrecedenceTime(). Otherwise, the Motion Command will prevent actions forever. Use ArRobot::getDirectMotionPrecedenceTime() to see how long a Motion Command takes precedence once set.
</p>
</details>
