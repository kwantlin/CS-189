# PSET 2: Wander
Kat^2

## Organization

### Main files

`wander-MIN.py` is our final algorithm, using the MIN logic (explained below)

`wander-MEAN.py` utilized our MEAN logic

### Archived files

`archive/wander-MEAN-backup.py` has some other code we used to experiment

`archive/wander.py` is the lab 2 solutions code

`archive/wander-text.py` and `textbook.py` are experiments with the example code in PRR

`archive/depth_log.txt` is a sample of the raw output of the camera


## Logic

We created two algorithms: `MEAN` and `MIN`. Then we tested both and picked the one that seemed to work better, the `MIN` algorithm.

`MEAN` took the `segmented_depth_mask` array and simply counted the number of non-zero values (besides some rows at the bottom/floor). Thus it is essentially an average of how close the frame is.

`MIN` takes the raw `cv_image` array and records the minimum value (again, besides some rows at the bottom).

Then, we told the robot to go, with a slight CCW angular velocity (to wall follow) when the `MEAN` or `MIN` value is above a threshold, and to rotate CW if not. This way, we are able to move while rotating until the threshold is violated, and then rotate away from the perceived obstacle.

For both algorithms we decided to take out rows at the bottom because we found the depth scanner really had trouble with the floor and it often gave confusing and contradictory output. We decided not to segment/exclude columns because we found it didn't increase the accuracy, and in fact having more data (all the columns) seemed to be more helpful.


