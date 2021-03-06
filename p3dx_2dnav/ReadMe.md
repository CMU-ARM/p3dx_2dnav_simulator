# p3dx_2dnav
Author (until September 2018): Joe Connolly, jconnolly171@gmail.com
## Running Simulations
1) To run all simulations and output csv data in the /results folder, run ```rosrun p3dx_2dnav final_automated_simulations.py```. **IMPORTANT: Make sure you empty the simulations folder before you run this file.** Otherwise, data analysis will not work properly.

2) To run only one specific simulation, change lines 212-213 in ```run_one_simulation.py``` to the desired index of the run (as according to the map), planner (robot_only or coupled), and trajectory (as according to the map). Then, save and rosrun the file.

## Important File Parameters (in /yamls folder)
1) In global_planner_params, ```cycles``` governs how many search cycles the global planner takes before ending planning.

2) In local_planner_params, ```pdist_scale```, ```gdist_scale```, and ```occ_dist_scale``` determine the weights for how closely the robot follows the path and avoids obstacles. **These parameters still need some tuning**

3) In costmap_common_params, ```inflation_radius``` determines how far obstacles are inflated in the costmap. Seems reasonable for now but feel free to change, as it changes the global path planned.

## Changing Starting Positions
1) If, for some reason, a starting position for a simulation run needs to change, simply roslaunch one of the basic launch files (robot.launch or coupled.launch) and joystick navigate the robot (after hitting start on the grey xbox controller) to the new desired starting position. Then, ```rosrun p3dx_2dnav storeposes.py``` and enter the index of the starting position you want to replace (according to the indices on the map).

2) If you want to add a starting position, do the same as above, but instead entering the next available index when running storeposes. In addition, you will need to add to the variable runs_per_planner in analyze_data.py and cycles.py, as well as account for the change in final_automated_simulations.py lines 271-310 by changing the index limits associated with each trajectory (i.e. trajectory 1 is run indices 0-5 (so the if statement has "index<6"), trajectory 2 is indices 6-7 (so the elif statement has "index<8"), etc.)

## Changing Goal Positions
If you want to change the goal positions, rostopic echo the clicked_point topic to find your desired new goal position in rviz (using Publish Point in the gui) and copy/paste the new x and y values into the goals.json file in the /json folder

**NOTE: I reduced the number of endgoal options so that the robot_only planner would not consider paths that it calculated to be within walls. To add all of the original goals back, look back a couple of commits to the "updated parameters" commit in json/goals.json and copy/paste into the local file**
