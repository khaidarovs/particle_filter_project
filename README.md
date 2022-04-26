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
- Location: Implemented inside initialize_particle_cloud function on line 155
- We used a helper function in order to get the location of the maze on the map. The get_inside_map(n, map) uses the map's data attribute, which returns 0 for locations inside the maze, and returns a list of locations on the map that are inside the maze. We then use these locations in order to randomly assign x and y values for the position of each particle. In order to get the orientation, we multiply 2pi by a random number between 0 and 1, to get a random orientation. We then convert the obtained theta to quaternion and set the orientation of the particle.

#### Movement model
- Location: Implemented inside update_particles_with_motion_model on line 338
- For this step we calculated how much the robot has moved in x and y directions by looking at the difference between their last and current positions. We did the same for orientation. Once we got all the values, we added the differences to the x and y position values and the orientation value of the particle. The process was repeated for all particles in the cloud. 

#### Measurement model
- Location: Implemented inside update_particle_weights_with_measurement_model on line 317
- For this step, we initially considered all 360 degrees, however after testing, we found that using multiples of 45, for the angles to look at, was a better choice in terms of efficiency and accuracy. We then used the likelyhood_field_range_finder_model algorithm to compute the weights for each particle. We set Zhit to 0.8, Zrand and Zmax to 0.1 (each) in order to account for some noise in the environment. 

#### Resampling
- Location: Implemented inside resample_particles on line 213
- In this step, we used np.random.choice in order to resample particles, taking into account the different weights of particles, specified in the weight list. As mentioned in the code, we then deepcopy the resampled particles back into the particle cloud. 

#### Incorporation of noise
- Location: Implemented inside update_particles_with_motion_model on line 355
- We added noise to the distance and orientation values that we should update the particles' location and orientaion by. We sampled from normal distribution with standard deviation of (0.1 * diff calculated), which we thought would result in a suitable amount of error.

#### Updating estimated robot pose
- Location: Implemented inside update_estimated_robot_pose on line 295
- For this step, we averaged the x and y positions of the particles on the grid. We then averaged the orientations of the particles on the grid, and assigned these averaged values for x, y  and theta to the estimated robot position and orientation. 

#### Optimization of Parameters
- Location: Optimized the number of angles in update_particle_weights_with_measurement_model as well as noise in update_particles_with_motion_model
- As it was mentioned above, some of the parameters we optimized were the angles at which we looked at the scan values of the turtlebot. Using multiples of 45 instead of all 360 degrees resulted in a more efficient and accurate performance of the localization algorithm. We left the number of particles at 10,000 for the same reason. In terms of noise, we incorporated noise in the motion model and added approximately 10% variation in each of the motion directions - x, y and orientation theta. After experimenting we came to a conclusion that this was a suitable amount of noise. 

### Challenges
- A big challenge was understanding how the topics such as OccupancyGrid() work and how to correctly use their attributes. This affected the difficulty of initializing the particle cloud and ensuring that the locations of the particles were randomly spread around the map of the maze. To overcome this challenge, we looked at the data attribute of the map, which is a 1D array of the map in row-major order. The locations within the maze had a value of 0 in data. Thus we took out these locations into a separate list, and chose random locations for x and y values from that list. Another challenge was optimizing the parameters to make the model more accurate, and this was done through a series of experiments with physical turtlebots and calibration of the parameters. 

### Future Work
- If we had more time it would be useful to continue optimizing the parameters such as the number of particles, the number of angles we look at and so on, to make the model more accurate and efficient. It would also be useful to explore how this localization method works in other environments, such as bigger rooms or even whole building floors, and see what kind of noise gets introduced in such environments and what would be the best way to handle that. 

### Takeaways
- The main takeaway from this project is getting experience working with other ros topics such as Pose and OccupancyGrid and just being more exposed to the different ways a robot can be operated and the ways in which it can receive information from the environment. This will definitely be useful going forward since we will be now able to incorporate the knowledge of localization and robot's position into the next and final projects.
- Another takeaway was experience of working in pairs on a project like this. Working with physical turtlebots in pairs is definitely different than working on solely coding projects, but we managed to finish the project by splitting up the functions and then coming together to test them out and debug on the spot. This is a valuable experience that will definitely be useful in the remaining projects. 

### Gif


https://user-images.githubusercontent.com/93730296/165332450-8fa45f5e-e28a-42f3-b71a-eb8bd36e0cf6.mov





