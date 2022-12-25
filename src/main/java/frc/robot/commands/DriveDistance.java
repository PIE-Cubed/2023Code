// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import frc.robot.subsystems.Drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/**
 * Start of the DriveDistance class
 */
public class DriveDistance extends CommandBase {
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

    // Variables
    private double startX = 0.00;
    private double startY = 0.00;
    private double startHeading = 0.00;
    private double endX = 0.00;
    private double endY = 0.00;
    private double endHeading = 0.00;
    private double midX = 0.00;
    private double midY = 0.00;
    private double midHeading = 0.00;

    // Object creation
    private Drive drive;
    private Timer timer;

    // Command creation
    private Trajectory path;
    private SwerveControllerCommand swerveControllerCommand;

    /**
     * Constructor for the DriveDistance class
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveDistance(Drive drive, double[] startCoor, double[] endCoor) {
        // Localizes drive
        this.drive = drive;

        // Gets the starting values
        this.startX = startCoor[0];
        this.startY = startCoor[1];
        this.startHeading = startCoor[2];

        // Gets the ending values
        this.endX = endCoor[0];
        this.endY = endCoor[1];
        this.endHeading = endCoor[2];

        // Calculate the mid values
        midX = (this.startX + this.endX) / 2;
        midY = (this.startY + this.endY) / 2;
        midHeading = (this.startHeading + this.endHeading) / 2;

        // Instance creation
        timer = new Timer();

        // Declare subsystem dependencies
        addRequirements(drive);
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
                .setKinematics(Drive.DriveKinematics);

        // A trajectory to follow. Units are converted as needed.
        path = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(ftToM(startX), ftToM(startY), rotation2dDeg(startHeading)), // Start at the origin facing 0 direction
                new Pose2d(ftToM(midX), ftToM(midY), rotation2dDeg(midHeading)),       // Pass through a mid point
                new Pose2d(ftToM(endX), ftToM(endY), rotation2dDeg(endHeading))        // End end at the endPoint, facing in direction endHeading
            ),
            config);
        
        // Creates a SwerveControllerCommand
        swerveControllerCommand =
            new SwerveControllerCommand(
                path,
                drive::getPose, // Functional interface to feed supplier
                Drive.DriveKinematics,

                // Position controllers
                xPosController,  // Uses the created xPosController
                yPosController,  // Uses the created yPosController
                thetaController, // Uses the created thetaController
                drive::setModuleStates,
                drive);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Runs the swerve command and then idles
        swerveControllerCommand.andThen(() -> drive.teleOpDrive(0, 0, 0, false));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(path.getTotalTimeSeconds());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Prints that the command ran
        System.out.println("Moved to X" + endX + "Y" + endY);
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

    /**
     * ftToM()
     * <p>Converts feet to meters
     * 
     * @param feet
     * @return
     */
    private double ftToM(double feet) {
        return Units.feetToMeters(feet);
    }
}

// End of the DriveDistance class