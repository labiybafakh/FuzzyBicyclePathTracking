# FuzzyBicyclePathPlanning
This work is my bachelor thesis project that have been presented in 2018.

The microcontroller that was used is STM32F1.
Motor1 to motor3 are the code to control 3 motor on omnidirectional mobile robot. Those motors are in close-loop mode and subscribe the desired speed from PC.
Odometry data was gathered by odometry code and is sent to PC.
Omnidirectional mobile robot was controlled by using omnirobot code. It gathered odometry data to estimate the position and do a path planning by using bicycle path plannning.
The disturbance or error was minimized by using Fuzzy Logic Controller to enable the robot to recover.

