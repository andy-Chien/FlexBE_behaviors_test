# DCMA Behaviors  
This repo contains the test-specific states and behaviors of dcma planner.  
## Behaviors:  
**Test behaviors:**  
Test some fixed poses for one ur5 robot with dcma planner  
Can define the dcma planner action name, robot id, and plan mode  
Plan mode can switch between plan_only, offline, and online.  
**Dual arm behaviors:**  
Test some fixed poses for two ur5 robots with dcma planner  
**Three arms test:**  
Test some fixed poses for three ur5 robots with dcma planner  
## States:  
**Get pose state:**  
To get a fixed pose to move.  
Output: translation (x,y,z) and rotation (3x3 matrix)  
**Ur5 ik state:**  
Calculate IK of UR5.  
Input: translation (x,y,z) and rotation (3x3 matrix)  
Output: target joint values of robot  
**Planning state:**  
Send target joint values to the dcma planner  
Input: target joint values of robot  
Output: planned or executed joint trajectory depend on plan only or not  
**Robot move state:**  
Used to move robot in plan only mode  
Input: planned joint trajectory  

