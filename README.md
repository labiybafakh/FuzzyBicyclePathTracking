# FuzzyBicyclePathPlanning
This work is my bachelor thesis project that have been presented in 2018. It used ROS kinetic to facilitate the real-time communication and data distribution.

1. Hardware
---
The microcontroller that was used is STM32F1.
Motor1 to motor3 are the code to control 3 motor on omnidirectional mobile robot. Those motors are in close-loop mode and subscribe the desired speed from PC.
Odometry data was gathered by odometry code and is sent to PC.

2. System
---
Omnidirectional mobile robot was controlled by using omnirobot code. It gathered odometry data to estimate the position and do a path planning by using bicycle path plannning.
The disturbance or error was minimized by using Fuzzy Logic Controller to enable the robot to recover.

---
* Paper can be found here
> M. L. Afakh, M. I. Masudi, F. Ardilla, I. K. Wibowo and B. S. Marta, "Bicycle Path Planning on Omnidirectional Mobile Robot Using Fuzzy Logic Controller," 2018 10th International Conference on Information Technology and Electrical Engineering (ICITEE), Bali, Indonesia, 2018, pp. 237-241, doi: 10.1109/ICITEED.2018.8534842.