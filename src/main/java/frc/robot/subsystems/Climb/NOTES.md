# Lift Mechanism

Lift is powered by a single neo 1.1 powered both directions. Need to get the reduction from August. Assume 25:1 for now.

pivot mounted to lift stage. Pivot will rotate the robot. Also a neo 1.1 with a large reduction. Same as above.

## Use-case

raise lift an amount (TBD).
Drive shaft into the ladder.
fully lower lift.
pivot an mount (until robot is upside-down).
fully extend the lift(?)

## implementation notes

PID position control using the neos relative encoders.

Lift has 2 or 3 positions. Min, Max, and whatever height is needed to insert the bar. Hopefully that is just max, but probably not.

The pivot will use position control to flip the robot upside-down, 180 degrees. pid will be tricky here because of the balancing act.

Cascading 2 stage lift with 2" sprockets