# gazebo-pset-4b
Autonomous OG Mapper for Gazebo



## Running the simulation

1. Instantiate the world and spawn the turtlebot by launching GazeboMapper.launch file using the command below. Note the x and y specifies the position of the turtlebot in cartesian coordinates and yaw specifies the orientation of the turtlebot in radians i.e. 3.142 = 180 degrees. The world parameter specifies the path to a world file which we provide (open_playpen.world and playpen.world). Note: the path should be declared as a full path hence the need to invoke `pwd`.

`roslaunch GazeboMapper.launch x:=0 y:=0 yaw:=3.142 world:=$(pwd)/open_playpen.world`

2. From the burger menu launch Gazebo to view the world and the turtlelbot

3. Also launch the Graphical Tools to view OpenCV windows (when displaying a map using your code)

4. Launch the mapping program using the command below

`python mapper.py`

## VIDEO DEMOS

### Open Playground
https://youtu.be/FgfAEFpofII

### Office
https://youtu.be/2fZHpA97ano



## Process

### Depth Sensors

To develop my depth sensor model, I took one strip at around row 300 from the depth sensor image file, converted those distances to positions using the angle of view specs of the camera and the turtlebots current position, and then converted each of those blocked cells to blue. Every cell along the "slope" from the bot's current position to where it hits an obstacle is marked empty. I chose to only mark empty using the depth camera when it sensed an obstacle, for greatest accuracy.

### Smart Exploration

I used my find_empty function to search the center of the map and determine the first unexplored node. When the turtlebot is given an option to turn randomly when at a center aligned obstacle, it will choose to turn in the direction of that unexplored node, using the homing code provide in Lab 4b solutions. If I had had more time, I might have further refined with A* or tried to hone this for more "forward motion" scenarios. However, I noticed that if I applied this homing to more than just this scenario (for example, when the turn is chosen randomly for forward motion in this version of wander code), it led to more repeated traversal of explored territory. Given more time (I had another CS final project due today) I would have expanded on this, and perhaps make it workable by marking empty nodes even when an obstacle was not detected.
