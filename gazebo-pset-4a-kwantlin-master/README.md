# gazebo-pset-4a
Autonomous OG Mapper for Gazebo



## Running the simulation

1. Instantiate the world and spawn the turtlebot by launching GazeboMapper.launch file using the command below. Note the x and y specifies the position of the turtlebot in cartesian coordinates and yaw specifies the orientation of the turtlebot in radians i.e. 3.142 = 180 degrees. The world parameter specifies the path to a world file which we provide (open_playpen.world and playpen.world). Note: the path should be declared as a full path hence the need to invoke `pwd`.

`roslaunch GazeboMapper.launch x:=0 y:=0 yaw:=3.142 world:=$(pwd)/open_playpen.world`

2. From the burger menu launch Gazebo to view the world and the turtlelbot

3. Also launch the Graphical Tools to view OpenCV windows (when displaying a map using your code)

4. Launch the mapping program using the command below

`python mapper.py`

