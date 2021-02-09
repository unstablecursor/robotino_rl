I adjusted the launch file, it now gives the ROBOT variable to the program and starts the right Gazebo.
Pls ONLY start with launch file:

roslaunch my_package_tut2 launchfile.launch

(Otherwise the export variable will get ignored no idea why)


I had two main mistakes in my prev version:

1) I got confused with the three different sub/pub: cmd_vel, input/cmd_vel and pioneer/cmd_vel. In my prev version the keyboard directly published to gazebo and therefor values where not "filtered" trough my program leading to the behaviour that it only works if I press the button once and not more often.

2) The publish/subscriber order was not working correct

3) (Addition). I distinguish now between the different sides (left/right(in front)). I saw this in another solution and wanted to test it out. My prev version did not distinguish between the sides and if one side was to close every velocity got slower or stopped. I did not copy the solution! Only wanted to make it more smooth. I hope it is okay.
