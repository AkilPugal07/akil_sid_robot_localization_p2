# Robot Localization through Particle Filter
Akil Pugalenthi & Siddhant Kulkarni


## Project Goal
The goal of a particle filter is to answer the grounding question "Where am I?" In the context of a robot. A particle filter can be used when the robot has near-perfect information of its environment, but does not know where in the environment it is. Our goal was to implement several key functions in the particle filter, which is in pf.py. This includes key logic on how particles are initialized, how particles are selectively resampled, how particle's weights are updated based on odometry data, and more.

![Screenshot2](pictures/ss2.png)

*Figure 1*
## Approach

Our particle filter method maintains a set of particles, where each one is a guess of what the robot's pose (x, y, heading) is. Each iteration of the particle filtering methods goes through the following steps

1. Initialize particle cloud: A set of randomly generated particles are spread through out the map. Each of them is a guess of what a possible location of the robot could be.
2. Motion update: Check the odometry change of the Neato and convert into the robot's frame co-ordinates. 
3. Weight update: For each particle, we took 72 of the 360 laserscans and project them into the map frame from that particle's pose, and measure each points closes distance to the nearest wall. Particles whose projected points land on or near walls get high weights. (Double check)
4. Normalize the weights: Weights are scaled to sum to 1.
5. Resample: We draw a new set of particles based on their proportions ans add noise to avoid overlap. A fixed set of particles are always generated randomly to avoid the code converging at the wrong spot.
6. Estimate: We update the the best guess of the robot pose based on the weighted mean of the particles. Over time through multiple iterations, the particles cluster around the Neato's true position.


## Design Decisions

72 lidar beams instead of one:
Our first implementation scored each particle using only the single closest laser reading, passing it through a normal PDF centered on the robot’s actual reading. This was fast, but one beam says very little about where the robot is, since many different poses produce the same closest-wall distance. We switched to every fifth beam (beam_step = 5), which gives 72 of the 360 readings. We skip any reading that is infinite or zero, so the actual number of beams scored per update can be a little under 72.

Scoring by distance to the nearest wall:
TODO

Random Initialization across the whole map:
We decided to not start up a gaussian distribution of the particle cloud because it could cause the code to converge at the wrong spot. So we  sample 300 particles uniformly within the map's bounding box. We also reject particles that might land on an obstacle by only letting it populate on the free spaces of the map with a random heading from 0 to 2*pi. The tradeoff is slower initial convergence, since most particles start far from the robot. In exchange, the filter works from any starting position with no manual initial pose, which is also why we ignore the pose passed in from rviz.

Resampling with a reserve of random particles:
On each resample, we draw 290 particles in proportion to their normalized weights and add uniform noise so duplicate copies spread out. The remaining 10 particles (random_reserve, about 3% of the cloud) are placed randomly across the free map, similar to initialization. Pure weighted resampling can collapse onto a wrong cluster and never recover, because no particles remain near the true pose. The random reserve lets the filter recover from bad guesses and from the kidnapped robot problem. The cost is that these particles are almost always wasted once the filter has converged, so making this number adaptive is on our list of future improvements.

Weighted mean for robot pose:
We estimate the robot’s pose as the weighted mean of all particles, not the single highest-weight particle. The best single particle can jump around between iterations, while the mean uses the whole cloud and is smoother. For heading, we average the sine and cosine of each particle’s angle and recombine with atan2, since averaging raw angles breaks near ±π. The weakness is that if the cloud splits into two clusters (common early on in symmetric areas), the mean can land between them, where there is no real hypothesis. We accepted this because the cloud usually collapses to one cluster after a few updates. 


![diagram](pictures/diag.jpeg)

## Challenges
1. Converting between reference frames: This was challenging when implementing updates through odometry. Initially, we simply added added the odom delta to the particle, but this caused the particle cloud to drift in the wrong direction. We fixed it by rotating the delta into the robot’s previous frame, then rotating that robot-frame motion by each particle’s own heading before applying it. (double check)
2. Unit testing was difficult when using ros nodes, especially when there are multiple syntax or runtime errors.
3. Staying on the one beam method for too long: 
## Future Improvements
1. Getting more accurate information on sensors, could inform more accurate noise implementation.
2. Dynamically add random particles, not a fixed amount every iteration.
3. Add constants as Ros parameters for fine tuning.

## Testing
Since unit testing inside a ROS node was difficult, we verified our code with print statements and an assert while running the filter on bag files in rviz. Each step of the filter prints a short summary, like the odometry delta, the number of laser beams scored, the highest particle weight, and the estimated pose, so we could tell which step was misbehaving instead of only seeing that the final pose was wrong. The odometry delta print was the most useful, since it showed our first odometry update was moving particles in the odom frame instead of the robot's frame.


We also assert that the particle weights sum to 1 at the end of normalizing. This catches NaN or infinite weights as soon as they appear, and paired with a print for when every weight is zero, it told us when our weighting was too harsh for the filter to converge.

## Lessons Learned
1. Since noise is required for an implementation of a particle filter, this adds an element of stochasticity to the filter's result. This creates run-to-run variance, which could lead to two problems. It can lead to us re-running non-functional iterations of the particle filter in hopes that we got a bad iteration, and not trusting mostly functional versions. This creates difficulty in improving the filter, since it is sometimes hard to understand how much impact a given change actually made.
2. In large repositories, it is important to test individual modules of codebases to make sure you understand which portion of the code is not behaving properly.
3. More particles is not always better, sometimes leads to much slower convergence and the simulation could run out of time. This can apply to pretty much any parameter for other project, like clock speed for instance.
___

![Screenshot](pictures/ss1.png)
*Figure 2*
