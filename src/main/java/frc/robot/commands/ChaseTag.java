// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.CustomTables;
import frc.robot.subsystems.Drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Start of the ChaseTag class
 */
public class ChaseTag extends CommandBase {
    // Constraint Values
    public final double MAX_DRIVE_SPEED         = 3; // Meters per second
    public final double MAX_DRIVE_ACCELERATION  = 3; // Meters per second per second
    public final double MAX_ROTATE_SPEED        = Math.PI; // Radians per second
    public final double MAX_ROTATE_ACCELERATION = Math.PI; // Radians per second per second

    // Trapezoid PID Constraints
    private final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_DRIVE_SPEED, MAX_DRIVE_ACCELERATION);
    private final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_DRIVE_SPEED, MAX_DRIVE_ACCELERATION);
    private final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ROTATE_SPEED, MAX_ROTATE_ACCELERATION);

    // Tag to chase
    private final int TAG_ID = 0;
    private Pose3d lastResult;

    // Distance from tag to robot
    private Transform3d TAG_TO_GOAL = 
        new Transform3d(
            new Translation3d(1.0, 0.0, 0.0),
            new Rotation3d(0, 0, Math.PI)
        );

    // Profiled PID Controllers
    private ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0, X_CONSTRAINTS);
    private ProfiledPIDController yController = new ProfiledPIDController(0, 0, 0, Y_CONSTRAINTS);
    private ProfiledPIDController rotateController = new ProfiledPIDController(0, 0, 0, THETA_CONSTRAINTS);

    // Object Creation
    private Drive drive;
    private Supplier<Pose2d> poseProvider;

    /**
     * Constructor for the ChaseTag class
     *
     * @param subsystem The subsystem used by this command.
     */
    public ChaseTag(Drive drive, Supplier<Pose2d> poseProvider) {
        // Localizes variables
        this.drive  = drive;
        this.poseProvider = poseProvider;

        // Configures PID Controllers
        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        rotateController.setTolerance(Units.degreesToRadians(2));
        rotateController.enableContinuousInput(-Math.PI, Math.PI);

        // Declare subsystem dependencies
        addRequirements(this.drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //
        var robotPose = poseProvider.get();

        // 
        lastResult = null;

        // Resets the PID controllers
        rotateController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // 
        var robotPose2d = poseProvider.get();
        var robotPose =
            new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.00,
                new Rotation3d(0.00, 0.00, robotPose2d.getRotation().getRadians())
            );

        // Gets all results
        Pose3d[] results = CustomTables.getLatestResults();

        //
        if (results[TAG_ID] != lastResult) {
            //
            Pose3d result = results[TAG_ID];

            // Sets the last result
            lastResult = result;

            // 
            Pose3d cameraPose = robotPose.transformBy(Drive.CAMERA_OFFSET);

            //
            Transform3d camToTarget = new Transform3d(result.getTranslation(), result.getRotation());
            Pose3d targetPose = cameraPose.transformBy(camToTarget);

            //
            Pose2d goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

            //
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            rotateController.setGoal(goalPose.getRotation().getRadians());
        }
        
        if (lastResult == null) {
            // Stops the drive
            drive.stopWheels();
        }
        else {
            // Caclulates drive speeds
            double xSpeed = xController.calculate(robotPose.getX());
            double ySpeed = yController.calculate(robotPose.getX());
            double rotateSpeed = rotateController.calculate(robotPose2d.getRotation().getRadians());

            // Defines the range of powers
            final double CLAMP = 0.20;

            // Clamps the speeds
            xSpeed = MathUtil.clamp(xSpeed, -CLAMP, CLAMP);
            ySpeed = MathUtil.clamp(ySpeed, -CLAMP, CLAMP);
            rotateSpeed = MathUtil.clamp(rotateSpeed, -CLAMP, CLAMP);

            // Checks if the goal is reached
            if (xController.atGoal() == true) {
                xSpeed = 0;
            }
            if (yController.atGoal() == true) {
                ySpeed = 0;
            }
            if (rotateController.atGoal() == true) {
                rotateSpeed = 0;
            }

            // Drives
            drive.teleOpDrive(xSpeed, ySpeed, rotateSpeed, false);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && rotateController.atGoal();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stops the drive
        drive.stopWheels();
    }
}

// End of the ChaseTag class