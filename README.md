# What's all this, then?

This is heavily-commented example for various common WPILib
activities. It's consolidated from several years of experience
on Team 3373, and includes:

* A variety of handy utility classes and methods
* Implementations of flywheel and arm subsystems 
* Wrappers for Limelight usage and simulation
* Some high-level swerve- and vision-related behavior

It's meant to be copied-and-pasted into a current year project.
That's pretty low-budget, but ... it hasn't been turned into a 
vendor dependency out of paranoia. we always wind up having to
debug/tweak stuff, and it's much easier to do that when the 
source code is part of the project.

# TODO

* It might be useful to simulate wheels not turning immediately
in SwerveChassisSim


* Figure out how to tell for sure whether the Limelight is 
targeting an AprilTag (probably via "pipeline type"?)


* Figure out how to deal with the Limelight having multiple tags
in view at the same time


* Example velocity-based subsystem using hardware PID instead of 
software for high-velocity accuracy (e.g. for shooter)


* Example subsystem based on amperage limits like our 2025 
Reefscape climber


* Any pertinent examples around QuestNav?


* Updating odometry takes a fair amount of CPU cycles; we might 
want to limit ourselves to one estimator (vision only) in the 
competition?


* Our Limelight layer allocates a bunch of objects 
(LimelightTarget and  RawFiducial); can/should we limit this?