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
    public final double MAX_AUTO_SPEED                = 1; // Meters per second
    public final double MAX_AUTO_ACCELERATION         = 1; // Meters per second per second
    public final double MAX_AUTO_ANGULAR_SPEED        = Math.PI; // Radians per second
    public final double MAX_AUTO_ANGULAR_ACCELERATION = Math.PI; // Radians per second per second

    // xPosController variables
    private final double xP = Drive.MAX_DRIVE_SPEED / 2;
    private final double xI = 0.05;
    private final double xD = 0.00;
    private PIDController xPosController;

    // yPosController variables
    private final double yP = Drive.MAX_DRIVE_SPEED * (1 / 2);
    private final double yI = 0.05;
    private final double yD = 0.00;
    private PIDController yPosController;

    // thetaController variables
    private final double thetaP = Drive.MAX_ROTATE_SPEED * (1 / (2 * Math.PI));
    private final double thetaI = 0.05;
    private final double thetaD = 0;
    private ProfiledPIDController thetaController;

    // thetaController constraints
    private final TrapezoidProfile.Constraints thetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            MAX_AUTO_ANGULAR_SPEED,
            MAX_AUTO_ANGULAR_ACCELERATION
        );

    // Integator ranges
    private final double X_I_MAX = 0.1;
    private final double X_I_MIN = -1 * X_I_MAX;
    private final double Y_I_MAX = 0.1;
    private final double Y_I_MIN = -1 * Y_I_MAX;
    private final double ROTATE_I_MAX = 0.1;
    private final double ROTATE_I_MIN = -1 * ROTATE_I_MAX;

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

        // Create the PID Controllers
        createPIDControllers();
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
        this.points   = pointsFromCoordinates(coorArray);

        // Instance creation
        timer = new Timer();

        // Create the PID Controllers
        createPIDControllers();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                MAX_AUTO_SPEED,
                MAX_AUTO_ACCELERATION
            ).setKinematics(drive.swerveDriveKinematics); // Add kinematics to ensure max speed is actually obeyed

        // A trajectory to follow. Units are converted as needed.
        path = TrajectoryGenerator.generateTrajectory(
            points,
            config
        );
        
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
        swerveControllerCommand.andThen(
            () ->
            drive.teleopDrive(0, 0, 0, false)
        );
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
     * Creates the PID Controllers for this class.
     */
    private void createPIDControllers() {
        // Create the PID Controllers
        xPosController  = new PIDController(xP, xI, xD);
        yPosController  = new PIDController(yP, yI, yD);
        thetaController = new ProfiledPIDController(thetaP, thetaI, thetaD, thetaControllerConstraints);

        // Limit the Integrator Ranges
        xPosController.setIntegratorRange (X_I_MIN, Y_I_MAX);
        yPosController.setIntegratorRange (Y_I_MIN, Y_I_MAX);
        thetaController.setIntegratorRange(ROTATE_I_MIN, ROTATE_I_MAX);
    }

    /**
     * Generates a list of Pose2d points from a 2d array of double values.
     * 
     * @param listOfPoints
     * @return A list of Pose2d points
     */
    private List<Pose2d> pointsFromCoordinates(double[][] listOfPoints) {
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