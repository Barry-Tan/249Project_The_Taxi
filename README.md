# 249Project_The_Taxi
Idea:

![alt text](https://github.com/Barry-Tan/249Project_The_Taxi/blob/main/idea.png)

Installation:
1. Intall Carla Simulator
2. Install requirement.txt
3. Run carlaUE4.exe
4. First run generate_traffic.py (this will simulate a traffic where 30~ cars will generate and turn on autodriving mode)
5. Run Car_GUI.py and selecy destination & pick up location & "call service"
6. Should generate a ego vehicle, mini cooper and front camera window showing operation

Current Stage:

1. Car_GUI.py -> [Main Application] include our Front end and ego taxi follow A* route waypoints in carla and execute the completed flow chart architure.

2. Yolov_test.ipynb -> include runing live inference with ego taxi with YOLOV5

3. PID_testing.py+contoller2d2.py+ctuils.py -> focus on implementing PID controller to the program

![alt text](https://github.com/Barry-Tan/249Project_The_Taxi/blob/main/map1.png)

Example path:

![alt text](https://github.com/Barry-Tan/249Project_The_Taxi/blob/main/path.png)


Reference:
https://github.com/vadim7s/SelfDrive/tree/master
https://github.com/ultralytics/yolov5
https://github.com/abwerby/Model-Predictive-Control-Carla/tree/master
