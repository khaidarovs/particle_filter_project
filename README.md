# particle_filter_project

## Implementation plan

### Team members:
Sanzhar Khaidarov, Seha Choi

#### initialize_particle_cloud

- To initialize particle cloud, we would randomly assign the particles a location on the map. 

#### update_particles_with_motion_model

- Similar to the exercise in Class 5, we would update the location of the particles by moving them in the same direction and by the same distance as the the turtlebot. 

#### update_particle_weights_with_measurement_model

- Once again, similar to the exercise we did in class, after receiving robot's laser scan data, we would compare that with the laser scan data of each particle and then give the highest weighting to the readings that are closest to that of the turtlebot. 

#### normalize_particles & resample_particles

- In order to normalize the weightings of the particles, we would add up all the weightings and then divide each separate weighting by the total, thus normalizing it. We would resample the particles with the highest weighting, probably a weighting that is at least 2 times greater than the majority of other particles, and leave out particles with the lowest weightings (we would probably have to determine a more specific threshold when we get to experiment with the actual model and turtlebot).

#### update_estimated_robot_pose

- We would update the robot's pose to wherever the highest concentration of particles is (thus the particles that were given the highest weighting in the previous step). 

#### Incorporating noise

- We would probably have to come up with some error that the turtlebot will operate within. 

### Timeline

- Lab C: Build and save a map of the maze
- Before Class 7: Write code for initializing particle cloud and updating particles with motion model
- Lab D: Update particles with measurement model
- Before Class 8: Write code for normalizing particles, resampling them and updating the robot's pose
- Weekend before it's due: work with noise and perform some final corrections



