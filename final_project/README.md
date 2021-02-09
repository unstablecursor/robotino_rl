
Use: roslaunch rto_bringup_sim robot.launch  and 

rosrun final_project reinforcmentLearning.py


The Script resets the world (only in simulation possible)


Important:
In this Config the scanner is set to the non-gpu version! This is because my linux would give -inf otherwise.
The rto_simulation package is inside pcimr_tutorial_01 as I had to adjust the launch file and add new worlds.
The Folder Oval1 hast the model for the track in OvalWorld1.world. If the world has problems loading the track re-import from the folder or make a new model with the Oval.dae file ate the root of this repo.



roslaunch finalProject finalProject.launch
