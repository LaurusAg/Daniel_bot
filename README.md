## DANIEL ROBOT PACKAGE DEVELOPMENT 

This is a common space for the best laurus engineers to develop the best ROS code (or whatever you can do..)

Download locally and whenever a push is made, please write a descriptive message.

Cheers!



## Initial steps:  

  - Open a new terminal (stay in the home directory).
 
    >mkdir dev_ws   
     cd dev_ws/  
     mkdir src  
     cd src/  
     git clone https://github.com/LaurusAg/Daniel_bot.git  
  - Now you're ready to go! to test it, go back to dev_ws and build with colcon using the symlink (symlink will make your life easier because u don't have to build every time you make a change). 
    > cd ..   
     colcon build --symlink-install  
     source install/setup.bash  
     ros2 launch daniel_bot rsp_launch.py
     
  -you should see this output:

   > [robot_state_publisher]: got segment base_link  
     [robot_state_publisher]: got segment bicep_link  
     [robot_state_publisher]: got segment gantry_link  
     [robot_state_publisher]: got segment world


that's it!
