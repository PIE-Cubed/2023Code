// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;;

/**
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls). 
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // CONSTANTS
    private final double MAX_AUTO_ACC   = 2; // Meters per second per second
    private final double MAX_AUTO_SPEED = 2; // Meters per second

    // The robot's subsystems
    private final DriveSubsystem robotDrive = new DriveSubsystem();

    // Object creation
    private Controls controls;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Instance Creation
        controls = new Controls();

        // Configure default commands
        robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () ->
                robotDrive.drive(
                    controls.getDriveX(),
                    controls.getDriveY(),
                    controls.getRotatePower(),
                    controls.toggleFieldDrive()),
                robotDrive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                MAX_AUTO_SPEED,
                MAX_AUTO_ACC)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.DriveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory path =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, rotation2dDeg(0)),

                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),

                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, rotation2dDeg(0)),
                config);

        var thetaController = new ProfiledPIDController(Constants.thetaP, 0, 0, Constants.thetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                path,
                robotDrive::getPose, // Functional interface to feed supplier
                Constants.DriveKinematics,

                // Position controllers
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                thetaController,
                robotDrive::setModuleStates,
                robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        robotDrive.resetOdometry(path.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false));
    }

    /**
     * rotation2dDeg()
     * <p>Creates a Rotation2d instance using degrees passed to the method
     * @param degrees
     * @return Rotation2d
     */
    private Rotation2d rotation2dDeg(double degrees) {
        double rad = Math.toRadians(degrees);
        return new Rotation2d(rad);
    }
}

// End of the RobotContainer class