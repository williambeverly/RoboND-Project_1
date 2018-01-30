## Project: Search and Sample Return

---

**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./writeup_images/transformed.png
[image2]: ./writeup_images/thresholded.png
[image3]: ./writeup_images/rock_transform.png
[image4]: ./writeup_images/gif.gif

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
The camera image was transformed to a top-down image, and a mask was created, in order to represent the boundaries of the transformed image, resulting in:

![alt text][image1]

To identify the road (light surface, an RGB colour threshold of (160,160,160) was utilised, resulting in:

![alt text][image2]

Anything below the threshold is considered an obstacle - with exception of the rocks. Collectable rocks were identified by transforming the RGB to HSV colour space, and applying a lower and upper threshold of (20,100,100) and (40,255,255) respectively. The image was then transformed to a top-down view, resulting in:

![alt text][image3]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 
A video of the output is stored in the 'output' folder. The gif is shown below:

![alt text][image4]

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and provide an explanation in the writeup of how and why these functions were modified as they were.

Modifications to drive_rover.py in RoverState classs (line 80 to 97):
* to bias unexplored territory, included an 'explored' map - initially all ones. As Rover explores, values are set to zero, and any value that is present in the worldmap and is navigable and unexplored (explored==1), effectively doubles these points, so that the average steering angle vector is biased towards unexplored territory.
* added pitch and roll tolerance values
* added some intelligence to store transformed values, override normal navigation and hold average velocity and average steering angle over 15 seconds, with an update rate of 1Hz.

Modifications to perception.py:
* included a world_to_pix function (line 99), required to transform the newly defined explored map to rover pixels.
* included a mask as an additional return value in perspect_transform function (line 124)
* included rock_thresh function that identifies rocks based on their yellow colour

Explanation of perception.py:
* warp front camera image to top-down view (line 152)
* apply colour threshold to return binary matrix with navigable area (line 154)
* invert the navigable terrain and apply a mask of visible area (line 156) to identify obstacles
* identify rocks from the warped image (line 157)
* update vision_image for rover based on the obstacles, rocks and navigable terrain (lines 162-164)
* convert to Rover-centric co-ordinates (lines 166-168)
* convert Rover-centric to World co-ordinates for obstacles, rocks and navigable terrain (lines 175-177)
* apply pitch and roll tolerance limits, and only update the worldmap if the values are within the acceptable tolerances (lines 184-187)
* calculate bias x and y points to be added to the existing x and y pixels (line 192)
* concatenate the x and y vectors and calculate the polar co-ordinates (dist, angles) of each x/y pair (lines 193-195)
* update rover navigation (lines 200-202)

Modifications to decision.py:
* defined two new modes of 'rover_stuck' and 'circular_motion' and added some logic to test whether rover is stuck, or travelling in circular motion (lines 10-22)
* implemented recursive logic that if rover is stuck (detected after 15 seconds each time), then turn by 30 degrees to the right, and try to go forward again (lines 26-41)
* implemented logic to detect circular motion - when detected, rover is stopped and turned towards the original starting position, and then instructed to override normal behaviour, and advance towards the origin for 5 seconds (lines 42-76)

Explanation of decision.py:
* simplistic state machine, with 4 modes: rover_stuck, circular_motion, forwards and stop, and should only be in one of those modes at any given time, provided there are navigation angles (set in perception.py)
* the forward and stop are the dominant modes, and orientate the rover by taking the average of all the navigable points, and setting the steering angle to the limited average
* throttle_set has been left at 0.2

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.

The Simulator settings utilised for my run are shown below. My frames per second varied from about 18 - 36 FPS.

![alt text][image5]

My rover does navigate autonomously, and can quite repeatably obtain above 40% mapped, with 60% fidelity. Most of the approach has been discussed above, so here, I will simply talk about some issues faced, and some potential improvements I would make.

Issues:
* rover can still become stuck on a rock, or navigate itself into a corner.
* the technique utilised to bias unexplored terrain does not prevent the rover from revisiting previously visited points - in fact the technique only works if there is indeed some unexplored terrain infront of it.
* rover does not have intelligence to collect samples, although the positions of the samples are being stored correctly

Improvements:
* at the start, do a full 360 rotation to determine the highest amount of navigable terrain that can be seen, and create a hierachal list of positions and orientations to return to.
* I am of the opinion the general image based searching is fine for navigating larger areas, but I would like to implement an A* algorithm for better navigating around obstacles and smaller spaces. This may also have the benefit of being able to set a goal position when becoming stuck, or being able to retrace a path back to a certain point, or indeed the initial position (when all samples have been collected)
* add logic that prevents revisiting an area, unless it is on return path to origin (and then potentially while on return, it sees unexplored areas, and switches to (to what I am now thinking I would call 'discovery' mode!)
* implement another mode called 'collection' that locks onto the position of the collectable rocks, to control the speed, position and orientation to collect the samples
