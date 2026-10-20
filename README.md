# Robot Localization through Particle Filter
Akil Pugalenthi & Siddhant Kulkarni


## Project Goal
The goal of a particle filter is to answer the grounding question "Where am I?" In the context of a robot. A particle filter can be used when the robot has near-perfect information of its environment, but does not know where in the environment it is. Our goal was to implement several key functions in the particle filter, which is in pf.py. This includes key logic on how particles are initialized, how particles are selectively resampled, how particle's weights are updated based on odometry data, and more.

![Screenshot2](pictures/ss2.png)

*Figure 1*
## Approach
At a high level, the particle filter first initializes particles randomly spread throughout the zone with obstacles (largely this covers the areas we have data on). These particles are positions and angles (poses) the robot could possibly be at on the map. The actual sensor readings from the robot are then compared to what each particle would see if the robot was at the particle's position. Based on how far off those readings are, the particle is given a weight, and particles with higher weights are more likely to persist to the next iteration (with some nosie). The idea is that over time, particles with the closest readings will form a clump in the map of where the robot actually is, and taking a weighted average/mode of the particle cloud should give a pretty good estimate over time of where the robot is. 

## Design Decisions
Our original implementation of the particle filter involved factoring in only the closest laser scan to the real robot. We then sample this value through normalpdf with a mean of the actual closest reading and standard deviation of typical sensor noise for a sensor of our type. The output p-value would then be our weight for the particle. This ended up sacrificing too much accuracy for speed, and we decided to use 72 (every fifth of the 360 lidar scans) to determine a particle's accuracy, as well as changing our method for calculating weight.


Our refined approach takes our 72 lidar readings and imposes it onto any given particle's pose. The readings are then converted to points in the map frame, and scored by how far they appear from the closest wall. For example, the best score would be given to a given laser scan reading if its position falls exactly on a wall, and if many of the 72 points end with high scores, the particle gets a high weight.


We also made the decision to save a subset of particles to be completely randomly distributed across the map regardless of sampling from the weighted distribution. We did this to allow the particle filter to recover from bad guesses, and provide opportunity for convergence in the event of robot kidnapping.
## Challenges
1. Converting reference frames was challenging when implementing updates through odometry. With the frame of odom being different than the robot itself, it required some tricky math to convert one to another which was not provided as a helper function
2. Unit testing was difficult when using ros nodes, especially when there are multiple syntax or runtime errors.
3. We committed a lot of time to our single laser scan implementation before realizing it could not provide the accuracy we needed. We could not tell if it was an implementation problem or a conceptual problem for a long time.
## Future Improvements
1. Getting more accurate information on sensors, could inform more accurate noise implementation.
2. Dynamically add random particles, not a fixed amount every iteration.
3. Add constants as Ros parameters for fine tuning.


## Lessons Learned


1. Since noise is required for an implementation of a particle filter, this adds an element of stochasticity to the filter's result. This creates run-to-run variance, which could lead to two problems. It can lead to us re-running non-functional iterations of the particle filter in hopes that we got a bad iteration, and not trusting mostly functional versions. This creates difficulty in improving the filter, since it is sometimes hard to understand how much impact a given change actually made.
2. In large repositories, it is important to test individual modules of codebases to make sure you understand which portion of the code is not behaving properly.
3. More particles is not always better, sometimes leads to much slower convergence and the simulation could run out of time. This can apply to pretty much any parameter for other project, like clock speed for instance.
___

![Screenshot](pictures/ss1.png)
*Figure 2*