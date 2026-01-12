# Code

## Vision Stuff

### Tower April Tags
**Blue**: 
Right Far: 18
Right Close: 27
Front Right: 26
Front Left: 25
Left Close: 24
Left Far: 21
Back Right: 19
Back Left: 20

**Red**:
Right Far: 2
Right Close: 11
Front Right: 10
Front Left: 9
Left Close: 8
Left Far: 5
Back Right: 3
Back Left: 4

### Andrew Notes (end of night 1):
Added a drive subsystem and the vision subsystem from Advantage Kit.

Have pose estimation using vision. Simulation is fully set up to support driving and vision targets.

Currently am not doing anything with april tags in terms of aligning, targeting, etc. Auto is possible with Pathplanner/Pathweaver/Choreo/etc due to pose estimation.

None of the physical drive/vision constants are defined due to not having a robot.

Photonvision on the orange pi (backpack) needs to be updated once it's released.

The NavX and Photonvision are currently using 2025 or previous libs because they aren't out yet (and the NavX is obselete?)

Rio needs to be updated once it has power and is able.

Vision improvements and Auto pathing can be completely done right now while we don't have a robot to work with. Once mechanisms are better defined we can write those.

### Andrew Notes (end of night 2):
Got physics calculations for shooter working (seems like). Constant auto-lock on the hub unless otherwise told.
Hood angle, turret angle, and flywheel velocity all calculated as needed. Hopefully takes motion into account.

Very tired implementation of climb code. Probably doesn't work. Skeleton of Intake code created.

TODO:
1. Intake subsystem (create)
2. Climb subsystem (sim testing and tuning)
3. Shooter subsystem (real testing and tuning)
4. Drive subsystem (real testing and tuning)
5. Vision subsystem (Photonvision setup, real testing and tuning)
6. Check library versions (in vendordeps) and update devices as needed.
7. Clean up scattered constants
8. Map out all of the devices
9. Probably forgetting things
