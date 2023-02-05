// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Collections;

import frc.robot.Drive;
import frc.robot.PoseEstimation;

import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/**
 * Start of the AutoDrive class
 */
public class AutoDrive extends CommandBase {
    // Variables
    private List<Pose2d> points;

    // CONSTANTS
    public final double MAX_AUTO_SPEED                = 3; // Meters per second
    public final double MAX_AUTO_ACCELERATION         = 3; // Meters per second per second
    public final double MAX_AUTO_ANGULAR_SPEED        = Math.PI; // Radians per second
    public final double MAX_AUTO_ANGULAR_ACCELERATION = Math.PI; // Radians per second per second

    // thetaController variables
    private final double thetaP = 0;
    private final double thetaI = 0;
    private final double thetaD = 0;

    // thetaController constraint
    public final TrapezoidProfile.Constraints thetaControllerConstraints =
        new TrapezoidProfile.Constraints(MAX_AUTO_ANGULAR_SPEED, MAX_AUTO_ANGULAR_ACCELERATION);

    // Create thetaController
    private ProfiledPIDController thetaController = new ProfiledPIDController(thetaP, thetaI, thetaD, thetaControllerConstraints);

    // xPosController variables
    private final double xP = 0.50;
    private final double xI = 0.00;
    private final double xD = 0.00;

    // Create xPosController
    private PIDController xPosController = new PIDController(xP, xI, xD);

    // yPosController variables
    private final double yP = 0.50;
    private final double yI = 0.00;
    private final double yD = 0.00;

    // Create xPosController
    private PIDController yPosController = new PIDController(yP, yI, yD);

    // Object creation
    private PoseEstimation position;
    private Drive          drive;
    private Timer          timer;

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
    public AutoDrive(Drive drive, PoseEstimation position, List<Pose2d> drivePoints) {
        // Localizes variables
        this.drive    = drive;
        this.position = position;
        this.points   = drivePoints;

        // Instance creation
        timer = new Timer();
    }

    /**
     * Constructor for the AutoDrive class.
     * 
     * @param drive
     * @param position
     * @param coorArray
     */
    public AutoDrive(Drive drive, PoseEstimation position, double[][] coorArray) {
        // Localizes variables
        this.drive    = drive;
        this.position = position;
        this.points   = poselistFromCoordinates(coorArray);

        // Instance creation
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                MAX_AUTO_SPEED,
                MAX_AUTO_ACCELERATION)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(drive.swerveDriveKinematics);

        // A trajectory to follow. Units are converted as needed.
        path = TrajectoryGenerator.generateTrajectory(
            points,
            config);
        
        // Creates a SwerveControllerCommand
        swerveControllerCommand =
            new SwerveControllerCommand(
                path,
                position::getVisionPose, // Functional interface to feed supplier
                drive.swerveDriveKinematics,

                // Position controllers
                xPosController,  // Uses the created xPosController
                yPosController,  // Uses the created yPosController
                thetaController, // Uses the created thetaController
                drive::setModuleStates);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Runs the swerve command and then idles
        swerveControllerCommand.andThen(() -> drive.teleopDrive(0, 0, 0, false));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(path.getTotalTimeSeconds());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Gets the end X and Y
        double endX = points.get(points.size() - 1).getX();
        double endY = points.get(points.size() - 1).getY();

        // Prints that the command ran
        System.out.println("Moved to X" + endX + "Y" + endY);
    }

    /**
     * Generates a list of Pose2d points from a 2d array of double values.
     * 
     * @param listOfPoints
     * @return A list of Pose2d points
     */
    private List<Pose2d> poselistFromCoordinates(double[][] listOfPoints) {
        // Creates an empty list
        List<Pose2d> list = Collections.emptyList();

        // Iterates through the array that was passed in
        for (int i = 0; i < listOfPoints.length; i++) {
            list.add(
                new Pose2d(
                    listOfPoints[i][0],
                    listOfPoints[i][1],
                    new Rotation2d( listOfPoints[i][2] )
                )
            );
        }

        return list;
    }
}

// End of the AutoDrive class