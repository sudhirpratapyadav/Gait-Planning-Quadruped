# Gait-Planning-for-Quadruped
1. Language, Frameworks -> Matlab, ROS
2. Algorithms/Knowledge -> Stability, Crawl gait, Cosine & Quintic trajectories, Kinematics
3. Things Done (7)
	1. Stability of Quadruped robot is analysed.
	2. Parameters like height of COM, stride length etc. were optimally selected based on analysis of workspace (range) of leg.
	3. Crawl gait was used for walking on plane surface.
	4. Different leg trajectories (in Walking direction) were compared and finally quintic polynomial trajectory was selected based on effectiveness.
	5. Kinematic simulation of quadruped was done to confirm above work.
	6. Quadruped was designed in solid works and was simulated to check collisions.
	7. Finally, a real quadruped was controlled by driving dinamixel motor through ROS. Quadruped was able to walk as same in simulation.
