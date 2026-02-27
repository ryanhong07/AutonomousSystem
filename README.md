Step 1: source /opt/ros/jazzy/setup.bash 
	colcon build 
	source install/setup.bash

Step 2: run the following terminals with following commands. (Dont forget to source ros2)

Terminal 1: 
	source install/setup.bash
        ros2 launch simulation simulation.launch.py
        
Terminal 2: 
	source install/setup.bash
	ros2 run simulation lantern_tracker.py 

Terminal 3: 
	source install/setup.bash
	ros2 run controller_pkg manual_pilot

Terminal 4: rviz2 (add camera feed from drone: semantics camera/image_raw/image & /realsensedepth/image/image)
