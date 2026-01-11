# Code

### Andrew Notes (end of night 1):
Added a drive subsystem and the vision subsystem from Advantage Kit.

Have pose estimation using vision. Simulation is fully set up to support driving and vision targets.

Currently am not doing anything with april tags in terms of aligning, targeting, etc. Auto is possible with Pathplanner/Pathweaver/Choreo/etc due to pose estimation.

None of the physical drive/vision constants are defined due to not having a robot.

Photonvision on the orange pi (backpack) needs to be updated once it's released.

The NavX and Photonvision are currently using 2025 or previous libs because they aren't out yet (and the NavX is obselete?)

Rio needs to be updated once it has power and is able.

Vision improvements and Auto pathing can be completely done right now while we don't have a robot to work with. Once mechanisms are better defined we can write those.
