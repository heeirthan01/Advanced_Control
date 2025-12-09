Course Repo for ECE238 (Advanced Control Lab)
The following files can be run to see results.

Project 1 (TO/MPC): 

Acrobot:
-mainacrobot.m -> Trajectory Optimization (TO) for acrobot case, with comparison to ODE simulated trajectory
-acrobot_lqr.m -> Swingup and stabilize for acrobot using LQR
-acrobotMPC.m -> .Swingup and stabilize using MPC

Mountain Car:
-mainmc.m -> TO and ODE45 simulation 
-LQRmc.m -> LQR for mountain car

Pendubot:
-mainpendubot.m -> TO and ODE45 simulation
-pendubot_lqr.m -> LQR for pendubot swingup + stabilization 

Project 2 (PFL/LQR of Acrobot -> recreating Spong95)
-acrobot_PFL.m -> Partial Feedback Linearization for acrobot swingup + stabiization using LQR
-zerodyn_ab.m -> Simulation of Acrobot zero dynamics

Project 3 (Swingup + Stabilization of Pendubot with state estimation)
-pendubotSimulink.slx (can be run on its own as simpb_init.m is set as InitFcn) -> shows the full swingup and stabilization using LQR.
-downrightpb.slx (can be run on its own as downrighttest.m is set as InitFcn) -> shows stabilization about [-pi/2;0;0;0].
-animatePB.m -> after running any of the two slx files, this file can be run to animate either case

References:
M. W. Spong, "The swing up control problem for the Acrobot," in IEEE Control Systems Magazine, vol. 15, no. 1, pp. 49-55, Feb. 1995, doi: 10.1109/37.341864.
keywords: {Robot kinematics;Actuators;Mobile robots;Orbital robotics;Control systems;Mechanical systems;Linear feedback control systems;Algorithm design and analysis;Robotics and automation;Equations},

S. Żak, “ECE 680 Model-based Predictive Control (MPC),” 2011. Available: https://engineering.purdue.edu/~zak/Second_ed/MPC_handout.pdf‌
