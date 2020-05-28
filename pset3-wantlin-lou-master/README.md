# PSET 3: Follow
Kat^2


## Organization

### Main files

#### Part A

`follow.py` is our final algorithm, with the median distance indicator being of prime consideration for detecting the distance to the sock

#### Part B

`follow-v2.py` and `follow-v3.py` contain our final algorithms, with the first prioritizing safety and the second focusing on the smoothest possible walk

### Archived files

N/A
`3a.launch`
`3a.py`
`lab2-solution.py`
`velocity_smoother.launch.xml`
`view_images_treshold.py`


## Logic

### Part A

Our follow algorithm searched for a broad range of HSV "green"values" to be sure of capturing the sock in every lighting, and drawing the bounding box (lab 2 solutions method), captured x,y coordinates of the upper left corner as well as the height h and width w. 

With the sock area captured on screen, we took advantage of the relative overlap between the depth camera and the RGB camera to search the bounding box-specified region of the depth camera image for depth. 

Given the variability of the readings, we chose to capturea few metrics about the region, including its perimeter and median/mean/min/max distance, not considering NaN values. 

In fact, given the depth camera's susceptibility to producing NaN values, we made sure to only consider median/mean/min/max distance when the bounding rectangle existed, firstly, and median distance (determined to be our most reliable metric of depth) was not determined from a NaN-filled bounding rectangle region. 

### Part B

Our follow algorithm built on bother `wander.py` from pset2 and also Part A's sock detection. 

In addition to searching for the green sock and drawing the bounding box on screen, we took advantage of Pset3a solutions to target a red circle over the sock, which would give us a more precise target to hone in on the sock's depth. Bump sensing was also added on top of 3a's implementation.

When our bot runs, it first takes into consideration if it has bumped an obstacle or not, and will back away a slight distance first. If it has, then it willl turn either away in the opposite direction if it has encountered a left or right bump obstacle, or turns in a random direction if the front sensor is triggered. 

If we have not bumped an obstacle, then we use our median distance to the sock, taken from within the bounding rectangle, along with the position as an angle from the vertical center of the image to implement a P-controller for the angle as well as the distance. (Inspired by PRR)

We rotate in the direction in which our angular error has deviated from the center, with that error normalized with a 1/100 constant. Similarly, we chose to normalize our distance error (calibrating it to remain at an optimal 0.5 m away) with a 1/2 constant. 


For `follow-v2.py` we also wanted to try to avoid obstacles before we bump them (and here, clearly, prioritize avoiding obstacles to following the sock). If we sense an obstacle in the left half of the screen passing our threshold, then we seek to avoid the left and rotate away over a range of 4. Otherwise, we check the right, and similarly rotate away. 

If we don't see any uniquely left or right obstacles, then we proceed with following. This tended to create quite a bumpy walk as it navigates circling an obstacle and also considers a stopped person within 0.5 feet to be an obstacle,a problem we have not yet worked out, though we hypothesize that a pixel count for green pixels might be a possible fix (stop the robot once the number of green pixels surpasses a certain threshold)

From here on out, our algorithms are identical.

First, we explore the case where the normalized distance is less than 0, which signals a NaN scenario or being lost. 
If we have noticed a duration of longer than two seconds since the last move, then if we have indeed lost the sock (bounding box is has perimeter less than the minimum threshold), we will publish beeps to signal this and rotate to look for the sock. Otherwise, we simply rotate. 

If we don't notice this long duration, then we repeat our last action.

If we have a normalized distance that is greater than our linear speed constant, signalling that we are farther from the sock than we'd like to be, then we move forward and note down the time of this most recent forward move. 


























