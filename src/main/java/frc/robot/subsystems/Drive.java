// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Robot;

import java.util.stream.DoubleStream;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Start of the Drive class
 */
public class Drive extends SubsystemBase {
    // Variables
    private boolean rotateFirstTime = true;
    private int     count           = 0;

    // Auto variables
    private long    timeOut;
    private static final int ON_ANGLE_COUNT = 10;

    // Drive Motor ID
    private final int FL_DRIVE = 10;
    private final int RL_DRIVE = 12;
    private final int FR_DRIVE = 14;
    private final int RR_DRIVE = 16;

    // Rotate Motor ID
    private final int FL_ROTATE = 11;
    private final int RL_ROTATE = 13;
    private final int FR_ROTATE = 17;
    private final int RR_ROTATE = 15;

    // Absolute Encoder ID
    private final int FL_ENCODER = 0;
    private final int RL_ENCODER = 1;
    private final int FR_ENCODER = 3;
    private final int RR_ENCODER = 2;

    // Absolute Encoder Offset (degrees)
    private final double FL_OFFSET = 309.8;
    private final double RL_OFFSET = 165.7;
    private final double FR_OFFSET = 248.0;
    private final double RR_OFFSET = -33.2;

    // Robot dimensions measured from the center of each wheel (inches)
    private static final double ROBOT_WIDTH_IN  = 18.0;
    private static final double ROBOT_LENGTH_IN = 30.128;

    // Robot dimensions measured from the center of each wheel (meters)
    private static final double ROBOT_WIDTH_M  = Units.inchesToMeters(ROBOT_WIDTH_IN);
    private static final double ROBOT_LENGTH_M = Units.inchesToMeters(ROBOT_LENGTH_IN);

    // Translations based off the wheel distances from the center of the robot (meters)
    private static final Translation2d FL_TRANSLATION = new Translation2d(-ROBOT_WIDTH_M / 2, ROBOT_LENGTH_M / 2);
    private static final Translation2d RL_TRANSLATION = new Translation2d(-ROBOT_WIDTH_M / 2, -ROBOT_LENGTH_M / 2);
    private static final Translation2d FR_TRANSLATION = new Translation2d(ROBOT_WIDTH_M / 2, ROBOT_LENGTH_M / 2);
    private static final Translation2d RR_TRANSLATION = new Translation2d(ROBOT_WIDTH_M / 2, -ROBOT_LENGTH_M / 2);

    // Kinematics Creation
    public static final SwerveDriveKinematics DriveKinematics =
        new SwerveDriveKinematics(FL_TRANSLATION, RL_TRANSLATION, FR_TRANSLATION, RR_TRANSLATION);

    // Robot swerve modules
    private final SwerveModule frontLeftWheel  = new SwerveModule(FL_DRIVE, FL_ROTATE, FL_ENCODER, true , false, FL_OFFSET);
    private final SwerveModule rearLeftWheel   = new SwerveModule(RL_DRIVE, RL_ROTATE, RL_ENCODER, true , false, RL_OFFSET);
    private final SwerveModule frontRightWheel = new SwerveModule(FR_DRIVE, FR_ROTATE, FR_ENCODER, false, false, FR_OFFSET);
    private final SwerveModule rearRightWheel  = new SwerveModule(RR_DRIVE, RR_ROTATE, RR_ENCODER, false, false, RR_OFFSET);

    // TeleOp Speeds
    private final double MAX_TELEOP_SPEED = 3; // Meters per second

    // Turn Controller
	private final double kP = 0.01; 
	private final double kI = 0.00;
    private final double kD = 0.00;
    private final double rotateToleranceDegrees = 2.0f;
    private PIDController rotateController;

    // Object Creation
    public static AHRS ahrs;
    private SwerveDriveOdometry odometry;

    /**
     * The constructor for the Drive class
     */
    public Drive() {
        // NavX
        try {
            ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            System.out.println("Error Instantiating NavX MXP: " + ex.getMessage());
        }

        // Resets the NavX
        ahrs.reset();

        // Connects to the NavX
        while (ahrs.isConnected() == false) {
            System.out.println("Connecting NavX");
        }
        System.out.println("NavX Connected");

        // Calibrates the NavX
        while (ahrs.isCalibrating() == true) {
            System.out.println("Calibrating NavX");
        }
        System.out.println("NavX Ready");

        // Zeros the NavX
        ahrs.zeroYaw();

        // Odometry class for tracking robot pose
        odometry = new SwerveDriveOdometry(DriveKinematics, rotation2dDeg(ahrs.getYaw()));

        // PID Controllers
        rotateController = new PIDController(kP, kI, kD);
        rotateController.setTolerance(rotateToleranceDegrees);
        rotateController.enableContinuousInput(-180.0, 180.0);
    }

    @Override
    public void periodic() {
        // Updates odometry periodically
        updateOdometry();
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
        odometry.resetPosition(pose, rotation2dDeg(ahrs.getYaw()));
    }

    /**
     * teleOpDrive()
     * <p>Method to drive the robot using joystick info.
     *
     * @param driveX Speed of the robot in the x direction (forward).
     * @param driveY Speed of the robot in the y direction (sideways).
     * @param rotate Angular rate of the robot.
     * @param fieldDrive Whether the provided x and y speeds are relative to the field.
     */
    public void teleOpDrive(double driveX, double driveY, double rotate, boolean fieldDrive) {
        var swerveModuleStates =
            DriveKinematics.toSwerveModuleStates(
                fieldDrive
                ? ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, rotate, rotation2dDeg(ahrs.getYaw()))
                : new ChassisSpeeds(driveX, driveY, rotate));
        
        // Limits the wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_TELEOP_SPEED);

        // Sets the desired states
        frontLeftWheel .setDesiredState(swerveModuleStates[0]);
        frontRightWheel.setDesiredState(swerveModuleStates[1]);
        rearLeftWheel  .setDesiredState(swerveModuleStates[2]);
        rearRightWheel .setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Limits the wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_TELEOP_SPEED);
    
        //  Sets the desired states
        frontLeftWheel .setDesiredState(desiredStates[0]);
        frontRightWheel.setDesiredState(desiredStates[1]);
        rearLeftWheel  .setDesiredState(desiredStates[2]);
        rearRightWheel .setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the NavX reading.
     */
    public void zeroHeading() {
        ahrs.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return ahrs.getYaw();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return ahrs.getVelocityZ();
    }
    
    /**
     * updateOdometry()
     * <p>Updates the odometry
     */
    public void updateOdometry() {
        odometry.update(
            rotation2dDeg(ahrs.getYaw()),
            frontLeftWheel.getState(),
            frontRightWheel.getState(),
            rearLeftWheel.getState(),
            rearRightWheel.getState()
        );
    }

    /**
     * rotation2dDeg()
     * <p>Creates a Rotation2d instance using degrees passed to the method
     * 
     * @param degrees
     * @return Rotation2d
     */
    private Rotation2d rotation2dDeg(double degrees) {
        double rad = Math.toRadians(degrees);
        return new Rotation2d(rad);
    }

    /**
     * teleopRotate()
     * <p>Rotates the robot with Z
     * 
     * @param rotatePower
     */
    public void teleopRotate(double rotatePower) {
        var swerveModuleStates =
            DriveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(0.00, 0.00, rotatePower));
        
        // Limits the wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_TELEOP_SPEED);

        // Sets the desired states
        frontLeftWheel .setDesiredState(swerveModuleStates[0]);
        frontRightWheel.setDesiredState(swerveModuleStates[1]);
        rearLeftWheel  .setDesiredState(swerveModuleStates[2]);
        rearRightWheel .setDesiredState(swerveModuleStates[3]);
    }

    /**
     * autoRotate()
     * <p>Rotates robot to inputted angle
     * 
     * @param degrees
     * @return
     */
    public int autoRotate(double degrees) {
        double rotateError;
        long currentMs = System.currentTimeMillis();

        if (rotateFirstTime == true) {
            rotateFirstTime = false;
            count = 0;
            timeOut = currentMs + 2500; // Makes the time out 2.5 seconds
        }

        if (currentMs > timeOut) {
			count = 0;
            rotateFirstTime = true;
            
			System.out.println("Auto rotate timed out");
            stopWheels();
            return Robot.DONE;
		}

		// Rotate
        rotateError = rotateController.calculate(ahrs.getYaw(), degrees);
        rotateError = MathUtil.clamp(rotateError, -0.5, 0.5);
		teleopRotate(rotateError);

		// CHECK: Routine Complete
		if (rotateController.atSetpoint() == true) {
            count++;            

			if (count == ON_ANGLE_COUNT) {
				count = 0;
                rotateFirstTime = true;
                rotateController.reset();
                stopWheels();                                
                return Robot.DONE;
            }
            else {
				return Robot.CONT;
			}
		}
		else {    
			count = 0;
            return Robot.CONT;
		}
    }

    /****************************************************************************************** 
    *
    *    TEST FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * stopWheels()
     * <p>Turns off all motors instead of turning wheels back to 0 degrees
     */
    public void stopWheels(){
        frontLeftWheel.setDriveMotorPower(0);
        frontRightWheel.setDriveMotorPower(0);
        rearLeftWheel.setDriveMotorPower(0);
        rearRightWheel.setDriveMotorPower(0);

        frontLeftWheel.setRotateMotorPower(0);
        frontRightWheel.setRotateMotorPower(0);
        rearLeftWheel.setRotateMotorPower(0);
        rearRightWheel.setRotateMotorPower(0);
    }

    /**
     * Tests the driving of all wheels
     */
    public void testWheel() {
        double power = 0.2;

        frontLeftWheel.setDriveMotorPower(power);
        frontRightWheel.setDriveMotorPower(power);
        rearLeftWheel.setDriveMotorPower(power);
        rearRightWheel.setDriveMotorPower(power);
    }
    
    /**
     * Tests the rotation of all wheels
     */
    public void testRotate() {
        double power = 0.2;
        frontLeftWheel.setRotateMotorPower(power);
        frontRightWheel.setRotateMotorPower(power);
        rearLeftWheel.setRotateMotorPower(power);
        rearRightWheel.setRotateMotorPower(power);
        System.out.println("Degrees: " + rearLeftWheel.getRotationPosition());
    }
    
    /**
     * Gets the average encoder of all wheels
     * 
     * @return averageEncoder
     */
    private double getAverageEncoder(){
        // Getting encoder values
        double frontRight = frontRightWheel.getEncoderValue();
        double frontLeft  = frontLeftWheel.getEncoderValue();
        double backRight  = rearRightWheel.getEncoderValue();
        double backLeft   = rearLeftWheel.getEncoderValue();

        // Creates a DoubleStream
        DoubleStream stream = DoubleStream.of(frontRight, frontLeft, backRight, backLeft);

        // Averages the DoubleStream and converts to a double
        double average = stream.average().getAsDouble();
        
        // Returns the average
        return average;
    }

    public void testWheelAngle() {
        // Use this to calibrate wheel angle sensors
        // Offset in wheel constructor should be the returned value * -1
        System.out.println("FL Offset: " + -frontLeftWheel.testWheelAngle());
        System.out.println("FR Offset: " + -frontRightWheel.testWheelAngle());
        System.out.println("RL Offset: " + -rearLeftWheel.testWheelAngle());
        System.out.println("RR Offset: " + -rearRightWheel.testWheelAngle());
    }
}

// End of the Drive class