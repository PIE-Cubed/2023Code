// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import frc.robot.Drive;
import frc.robot.PoseEstimation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/**
 * Start of the AutoDrive class
 */
public class AutoDrive extends CommandBase {
    // Variables
    private List<Pose2d> points;

    // CONSTANTS
    private final double MAX_SPEED                = 4; // Meters per second
    private final double MAX_ACCELERATION         = 1; // Meters per second per second
    private final double MAX_ANGULAR_SPEED        = Math.PI * 4; // Radians per second
    private final double MAX_ANGULAR_ACCELERATION = Math.PI / 4; // Radians per second per second

    // thetaController
    private final double thetaP = MAX_ANGULAR_SPEED * ((0.7) / Math.PI) / 5; ; // 1/0.7 Pi radians away --> full power
    private final double thetaI = 0;
    private final double thetaD = 0;
    private final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION);
    private ProfiledPIDController thetaController = new ProfiledPIDController(thetaP, thetaI, thetaD, thetaControllerConstraints);

    // xPosController
    private final double xP = MAX_SPEED / 5; // 1 meter away --> full power
    private final double xI = 0.00;
    private final double xD = 0.00;
    private PIDController xPosController = new PIDController(xP, xI, xD);

    // yPosController
    private final double yP = MAX_SPEED / 5; // 1 meter away --> full power
    private final double yI = 0.00;
    private final double yD = 0.00;
    private PIDController yPosController = new PIDController(yP, yI, yD);

    // Object creation
    private Timer   timer;
    private Drive   drive;
    private Field2d field;
    private PoseEstimation position;

    // Command creation
    private Trajectory path;
    private SwerveControllerCommand swerveControllerCommand;

    /**
     * Constructor for the AutoDrive class.
     * 
     * @param drive
     * @param position
     * @param drivePoints
     */
    public AutoDrive(Drive drive, PoseEstimation position, Field2d field, List<Pose2d> drivePoints) {
        // Localizes variables
        this.field    = field;
        this.points   = drivePoints;

        // Instance creation
        this.timer    = new Timer();
        this.drive    = drive;
        this.position = position;
    }

    @Override
    /**
     * Called when the command is initially scheduled.
     */
    public void initialize() {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                MAX_SPEED,
                MAX_ACCELERATION
            )
        // Add kinematics to ensure max speed is obeyed
        .setKinematics(drive.swerveDriveKinematics);

        // A trajectory to follow. Units are converted as needed.
        path = TrajectoryGenerator.generateTrajectory(
            points,
            config
        );

        // Push the trajectory to Field2d
        field.getObject("traj").setTrajectory(path);

        // Creates a SwerveControllerCommand
        swerveControllerCommand = new SwerveControllerCommand(
            path,
            position::getVisionPose, // Functional interface to feed supplier
            drive.swerveDriveKinematics,

            // Position controllers
            xPosController,  // Uses the created xPosController
            yPosController,  // Uses the created yPosController
            thetaController, // Uses the created thetaController
            drive::setModuleStates
        );
    }

    @Override
    /**
     * Called every time the scheduler runs while the command is scheduled.
     */
    public void execute() {
        // Runs the swerve command and then idle
        swerveControllerCommand.andThen(() -> drive.teleopDrive(0, 0, 0, false));
    }

    @Override
    /**
     * Returns true when the command should end.
     */
    public boolean isFinished() {
        return timer.hasElapsed(path.getTotalTimeSeconds());
    }

    @Override
    /**
     * Called once the command ends or is interrupted.
     */
    public void end(boolean interrupted) {
        // Gets the end X and Y
        double endX = points.get(points.size() - 1).getX();
        double endY = points.get(points.size() - 1).getY();

        // Prints that the command ran
        System.out.println("Moved to X " + endX + " Y " + endY);
    }
}

// End of the AutoDrive class