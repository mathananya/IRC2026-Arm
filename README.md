INITIAL SETTING UP THE ROVER:
in workingshit folder:
arm+drivecode :- for drive+arm esp
pivotcodeforarch:- flash into pivot esp
pivottestforachyutan :- quick debugging for elec where pivot encoder values are directly printed into arduino ide terminal


FOR CONTROLLING THE ROVER:
in controls folder :
controller8.py :- run this in a terminal
keyboard2.py :- run the environment with getch and then run this file , you have to do all the controlling from this file


P.S: Achyutan bro if you are confused with the environment and all just put that part into GPT and ask it to help .

For manually controlling the arm,
Ensure microRos is running and the topics are being echoed.
If you want to debug, send signal using given python scripts/ros nodes and then check ```ros2 topic echo arm_pwm_commands``` to see if data is going.
If data is being sent from there, the ros2 nodes are completely fine, check microROS implementation and the arduino code for debugging further.

FOR CONTROLLING THE ARM:

go to IRC2026-Arm/src/joystick_control/joystick_control
do ```
source ~/IRCFINALNOMORE....<some folder>/irc/bin/activate  #basically source the venv with getch.
python3 keyboardNode.py```
Now you can simply use W A S D keys to control the arm, W and S keys for the lower link and A and D keys for the upper link.

For manual joystick control,in the same directory, locate a file JoyAxesNode.py
do 
```
ros2 run joy joy_node
python3 JoyAxesNode.py
```
move the joystick as configured.

