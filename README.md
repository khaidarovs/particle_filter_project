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

- In order to normalize the weightings of the particles, we would add up all the weightings and then divide each separate weighting by the total, thus normalizing it. We would resample the particles with the proportional to the weights with replacement. We will check if after resampling the larger weighted particles are more preserved than the lower weighted particles.

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



