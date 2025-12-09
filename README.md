Course Repo for ECE238 (Advanced Control Lab) 

The files listed below can be run to see results.

Project 1 — Trajectory Optimization (TO) / Model Predictive Control (MPC)
Acrobot

mainacrobot.m – Trajectory Optimization (TO) for the acrobot, with comparison to an ODE-simulated trajectory.

acrobot_lqr.m – Swing-up and stabilization using LQR.

acrobotMPC.m – Swing-up and stabilization using MPC.

Mountain Car

mainmc.m – TO and ODE45 simulation.

LQRmc.m – LQR controller for the mountain-car system.

Pendubot

mainpendubot.m – TO and ODE45 simulation.

pendubot_lqr.m – LQR swing-up and stabilization for the pendubot.

Project 2 — PFL / LQR for Acrobot (Recreating Spong ’95)

acrobot_PFL.m – Partial Feedback Linearization for acrobot swing-up + LQR stabilization.

zerodyn_ab.m – Simulation of the acrobot zero dynamics.

Project 3 — Pendubot Swing-Up + Stabilization with State Estimation

pendubotSimulink.slx
Can be run directly (initialization handled by simpb_init.m). Shows full pendubot swing-up and LQR stabilization.

downrightpb.slx
Can be run directly (initialized via downrighttest.m). Demonstrates stabilization about the equilibrium 
[−π/2, 0, 0, 0].

animatePB.m
Run after either Simulink model to animate the corresponding pendubot trajectory.

References

Spong, M. W.
“The Swing Up Control Problem for the Acrobot.”
IEEE Control Systems Magazine, vol. 15, no. 1, pp. 49–55, Feb. 1995.
DOI: 10.1109/37.341864
Keywords: Robot kinematics, actuators, mobile robots, control systems, mechanical systems, linear feedback control systems, robotics and automation.

Żak, S.
“ECE 680 Model-based Predictive Control (MPC)”, 2011.
Available at: https://engineering.purdue.edu/~zak/Second_ed/MPC_handout.pdf
