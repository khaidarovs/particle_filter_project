# particle_filter_project

## Implementation plan

### Team members:
Sanzhar Khaidarov, Seha Choi

#### initialize_particle_cloud

- To initialize particle cloud, we would randomly assign the particles with random direction and a random location on the map. To test it we will display the particle elements on the map with their location and orientation. After we visualize them we could see the distribution of the particles.

#### update_particles_with_motion_model

- Similar to the exercise in Class 5, we would update the location of the particles by moving them in the same direction and by the same distance as the the turtlebot. We will test this via checking a couple of particles after they have been updated to see if it is the expected values.

#### update_particle_weights_with_measurement_model

- Once again, similar to the exercise we did in class, after receiving robot's laser scan data, we would compare that with the laser scan data of each particle and then give the highest weighting to the readings that are closest to that of the turtlebot. However, we will use all of the values from the LIDAR which is 360 values instead of 4 which is what we did in class.

#### normalize_particles & resample_particles

- In order to normalize the weightings of the particles, we would add up all the weightings and then divide each separate weighting by the total, thus normalizing it. We would resample the particles proportionally to the weights with replacement. We will check if after resampling, the larger weighted particles are more preserved than the lower weighted particles.

#### update_estimated_robot_pose

- We would update the robot's pose to the average of the pose of resampled particles (thus the particles that were given the highest weighting in the previous step). 

#### Incorporating noise

- We would probably have to come up with some error that the turtlebot will operate within. Then we will allow the particles to be updated to any value within the error threshold we have determined.

### Timeline

- Lab C: Build and save a map of the maze
- Before Class 7: Write code for initializing particle cloud and updating particles with motion model
- Lab D: Update particles with measurement model
- Before Class 8: Write code for normalizing particles, resampling them and updating the robot's pose
- Weekend before it's due: work with noise and perform some final corrections

## Writeup

### Objectives description
- The goal of this project was to use the particle filter localization method to find the location of a robot inside a maze. Our main goal was to get some experience working with probabilistic models to pinpoint the location of a moving robot.

### High-level discription
- To solve the problem of robot localization we used the approach described in class - the particle filter localization. We first randomly spread 10000 particles around the maze, each with uniform weight. Once the robot started moving, the locations of the particles were updated with the motion model, then the weights of the particles were recomputed based on the similarity between the particle scan readings and the robot scan readings. Finally the particles were resampled based on their recomputed weights. Eventually, the particles converged to the location of the robot in the maze. 

### Main Steps:

#### Initialization of particle cloud
- Location: lines 
- We used a helper function in order to get the location of the maze on the map. The get_inside_map(n, map) uses the map's data attribute, which returns 0 for locations inside the maze, and returns a list of locations on the map that are inside the maze. We then use these locations in order to randomly assign x and y values for the position of each particle. In order to get the orientation, we multiply 2pi by a random number between 0 and 1, to get a random orientation. We then convert the obtained theta to quaternion and set the orientation of the particle.

#### Movement model
- Location: lines
- For this step we calculated how much the robot has moved in x and y directions by looking at the difference between their last and current positions. We did the same for orientation. Once we got all the values, we added the differences to the x and y position values and the orientation value of the particle. The process was repeated for all particles in the cloud. 

#### Measurement model
- Location: lines
- For this step, we initially considered all 360 degrees, however after testing, we found that using multiples of 45, for the angles to look at, was a better choice in terms of efficiency and accuracy. We then used the likelyhood_field_range_finder_model algorithm to compute the weights for each particle. We set Zhit to 0.8, Zrand and Zmax to 0.1 (each) in order to account for some noise in the environment. 

#### Resampling







