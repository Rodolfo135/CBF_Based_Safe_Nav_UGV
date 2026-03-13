This project develops a robotic system for safe removal of wood waste in cluttered environments, with control-theoretic safety guarantees. The approach models a wheeled mobile robot with linear and angular velocity inputs and relies on local sensing to detect nearby obstacles and estimate their relative distance and orientation. Safety is enforced using Control Barrier Functions (CBFs) that maintain a minimum separation distance between the robot and obstacles. The resulting safety constraints are incorporated into a quadratic programming (QP) controller that adjusts the robot’s steering input while tracking a moving target. Simulation results demonstrate safe obstacle avoidance and target following, with future work focusing on multi-obstacle environments and more complex target trajectories.

Language utilized: Python

Primary libraries utilized:
- cyberwave python sdk
- scipy optimizer
- numpy
- CV2

Link to simulation video: https://youtu.be/gqld_x_bXoE

