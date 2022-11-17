# Hilltopper Robotics Software Library

HilltopperLib is a java library that provides common functionality between [First Robotics Competition](https://www.firstinspires.org/robotics/frc) games.

## Building

HilltopperLib uses [Gradle](gradle.org) to perform builds.

To build HilltopperLib, simply run `$ ./gradlew build` from the command line and the portable grale wrapper will build the library, no installation needed.

## Consuming

After a successful Gradle build, build artifacts can be found in `hilltopperlib/build`

To consume this library within a WPI robot project, do the following:
 1. copy the `libs` folder from `hilltopperlib/build` to the root directory of your robot project
 2. in the `build.gradle` file add `implementation files('libs/hilltopperlib.jar')` under the other `implementations` within `dependencies{}`

## AutoLib

AutoLib provides the class `DriveSegmentBaseCommand` that can be extended to allow a swerve drive robot to drive from one location to another.
