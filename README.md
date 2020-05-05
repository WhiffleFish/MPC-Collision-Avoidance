# MPC Collision Avoidance

Using Model Predictive Control to initiate a lane-change maneuver to facilitate obstacle avoidance  

Beginning with simple reference tracking maneuver
$$J_{1}=\sum_{i=1}^{c} (Y_{ref}-Y_{i})^{2}$$
![](Images/LaneChange.png)

Transitioning to robust model able to operate in a wide range of initial conditions
$$J_{2}=\sum_{i=1}^{c} u_{i}^{2} + K_{\psi}\psi_{i}^{2}$$
![](Images/frictionPlots.png)

![](Images/SpeedPlotsReduced.png)
