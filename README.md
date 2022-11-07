## How to setup and run my code
1. Plugin the B_L4S5I_IOT01A mbed board to the PC with USB cable that support USB signals.

2. Setup the circuit on mbed board and breadboard as shown in figure below.

![](https://github.com/lum33/hw3-5/blob/main/3-5-setup.jpg)

3. Keep mbed board balance on the desk as much as possible while running the program.

4. Flip mbed baord and press user button to decide current note length, then check result by pressing note button or length that printed in terminal.

## What are the results?
1. As shown in figure below, after the user button was pressed, 10 sets of roll/pitch/yaw will show in terminal first, then program will print the average value of these results. 

2. Note length is decided based on the latest average roll/pitch/yaw result:
    | Average value state | Note length |
    |---------------------|-------------|
    | Flip forward        | 0.125       |
    | Flip backward       | 0.25        |
    | rest                | 0.5         |
    | Flip sideway        | 1           |               
    
Note that: the blue color term that fits beside the average result indicates the current state space of mbed board.

![](https://github.com/lum33/hw3-5/blob/main/3-5-commended.JPG)
