// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    // Drive Motor ID
    private final int FL_DRIVE = 0;
    private final int RL_DRIVE = 0;
    private final int FR_DRIVE = 0;
    private final int RR_DRIVE = 0;

    // Rotate Motor ID
    private final int FL_ROTATE = 0;
    private final int RL_ROTATE = 0;
    private final int FR_ROTATE = 0;
    private final int RR_ROTATE = 0;

    // Absolute Encoder ID
    private final int FL_ENCODER = 0;
    private final int RL_ENCODER = 0;
    private final int FR_ENCODER = 0;
    private final int RR_ENCODER = 0;

    // Absolute Encoder Offset (degrees)
    private final double FL_OFFSET = 0;
    private final double RL_OFFSET = 0;
    private final double FR_OFFSET = 0;
    private final double RR_OFFSET = 0;

    // Robot swerve modules
    private final SwerveModule frontLeft  = new SwerveModule(FL_DRIVE, FL_ROTATE, FL_ENCODER, true , FL_OFFSET);
    private final SwerveModule rearLeft   = new SwerveModule(RL_DRIVE, RL_ROTATE, RL_ENCODER, true , RL_OFFSET);
    private final SwerveModule frontRight = new SwerveModule(FR_DRIVE, FR_ROTATE, FR_ENCODER, false, FR_OFFSET);
    private final SwerveModule rearRight  = new SwerveModule(RR_DRIVE, RR_ROTATE, RR_ENCODER, false, RR_OFFSET);

    // Object Creation
    private AHRS navX;
    private SwerveDriveOdometry odometry;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        // NavX
        try {
            navX = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            System.out.println("Error Instantiating navX MXP: " + ex.getMessage());
        }

        // Resets the NavX
        navX.reset();

        // Connects to the NavX
        while (navX.isConnected() == false) {
            System.out.println("Connecting NavX");
        }
        System.out.println("NavX Connected");

        // Calibrates the NavX
        while (navX.isCalibrating() == true) {
            System.out.println("Calibrating navX");
        }
        System.out.println("NavX Ready");

        // Zeros the NavX
        navX.zeroYaw();

        // Odometry class for tracking robot pose
        odometry = new SwerveDriveOdometry(Constants.DriveKinematics, rotation2dDeg(navX.getYaw()));
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(
            rotation2dDeg(navX.getYaw()),
            frontLeft.getState(),
            frontRight.getState(),
            rearLeft.getState(),
            rearRight.getState()
        );
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, rotation2dDeg(navX.getYaw()));
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot    Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates =
            Constants.DriveKinematics.toSwerveModuleStates(
                fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, rotation2dDeg(navX.getYaw()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        
        // Limitis the wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_TELEOP_SPEED);

        // Sets the desired states
        frontLeft .setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft  .setDesiredState(swerveModuleStates[2]);
        rearRight .setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_TELEOP_SPEED);
    
        frontLeft .setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft  .setDesiredState(desiredStates[2]);
        rearRight .setDesiredState(desiredStates[3]);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        navX.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return navX.getYaw();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return navX.getVelocityZ();
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

// End of the DriveSubsystem class