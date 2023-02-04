// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1732.autolib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import java.util.List;

/** Add your docs here. */
public abstract class DriveSegmentBaseCommand extends HilltopSwerveControllerCommand{
    private AutoSwerveDriveSubsystem _drivetrain;
    private Pose2d _initialPose;
    private boolean _resetPostion;
    private Rotation2d _startRotation;

    /** Creates a new Auto10Feet. */
    public DriveSegmentBaseCommand(AutoSwerveDriveSubsystem drivetrain,
                    List<Translation2d> waypoints,
                    Rotation2d startRotation,
                    Rotation2d endRotation,
                    boolean stopAtEnd,
                    boolean resetPosition) {
        super(getDefaultTrajectoryConfig(drivetrain, stopAtEnd),
                new Pose2d(waypoints.get(1), endRotation),
                drivetrain::getPose, // Functional interface to feed supplier
                drivetrain.getKinematics(),
                // Position controllers
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                getThetaController(),
                drivetrain::setModuleStates,
                drivetrain);
        _drivetrain = drivetrain;
        var firstWaypoint = waypoints.get(0);
        _initialPose = new Pose2d(firstWaypoint.getX(), firstWaypoint.getY(), startRotation); //getTrajectoryRotation(waypoints));
        _resetPostion = false;
        _startRotation = startRotation;
    }

    @Override
    public void initialize() {
        super.initialize();
        
        if (_resetPostion)
        {
            _drivetrain.zeroGyroscope(_startRotation.times(-1));
            _drivetrain.resetOdometry(_initialPose);
        }
        // if (!_resetPostion)
        //     _initialPose = new Pose2d(_drivetrain.getPose().getX(), _drivetrain.getPose().getY(), _drivetrain.getPose().getRotation());//_startRotation);
        // Reset odometry to the starting pose of the trajectory.        
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        // if (_resetPostion)
            // _drivetrain.zeroGyroscope(_endRotation);
    }

    private static ProfiledPIDController getThetaController() {
        var profileConstraints = new TrapezoidProfile.Constraints(
                0,//MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                0);//MAX_ANGULAR_ACCELERATION * Math.PI / 180 * 5);
        var thetaController = new ProfiledPIDController(7, 0, 0, profileConstraints);
        thetaController.enableContinuousInput(Math.PI * -1, Math.PI);
        return thetaController;
    }

    private static TrajectoryConfig getDefaultTrajectoryConfig(AutoSwerveDriveSubsystem drivetrain, boolean stopAtEnd) {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                0,//MAX_VELOCITY_METERS_PER_SECOND/3,
                0);//MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
        // Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(drivetrain.getKinematics());
        if (stopAtEnd)
            config.setEndVelocity(0.0);
        return config;
    }
}
