// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.EaseOfUse;
import frc.robot.CustomTables;

import java.util.stream.DoubleStream;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.math.kinematics.*;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Start of the Drive class
 */
public class Drive extends SubsystemBase {
    // Variables
    private Pose3d  prevPose;
    private double  prevTime;

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

    // Camera position relative to the center of the robot
    private static final double CAMERA_X_OFFSET = Units.feetToMeters(0);
    private static final double CAMERA_Y_OFFSET = Units.feetToMeters(0);
    private static final double CAMERA_Z_OFFSET = Units.feetToMeters(0);
    public static final Transform3d CAMERA_OFFSET = new Transform3d(
        new Translation3d(CAMERA_X_OFFSET, CAMERA_Y_OFFSET, CAMERA_Z_OFFSET),
        new Rotation3d()
    );

    // Object Creation
    private AHRS ahrs;
    private SwerveDriveOdometry odometry;
    private SwerveDrivePoseEstimator poseEstimator;

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

        // Odometry for tracking robot pose independant of vision sensors
        odometry = new SwerveDriveOdometry(
            DriveKinematics,
            EaseOfUse.generateRot2d( getHeading() ),
            getModulePositons(),
            new Pose2d()
        );

        // Create the standard deviations for how much the incoming data is trusted. Increase the values to trust the data less
        Vector<N3> stateStdDevs  = VecBuilder.fill(0, 0, 0); // x, y, and theta
        Vector<N3> visionStdDevs = VecBuilder.fill(0, 0, 0); // x, y, and theta

        // PoseEstimation for tracking robot pose with the help of vision sensors
        poseEstimator = new SwerveDrivePoseEstimator(
            DriveKinematics,
            EaseOfUse.generateRot2d( getHeading() ),
            getModulePositons(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs
        );

        // PID Controllers
        rotateController = new PIDController(kP, kI, kD);
        rotateController.setTolerance(rotateToleranceDegrees);
        rotateController.enableContinuousInput(-180.0, 180.0);
    }

    @Override
    public void periodic() {
        // Updates all pose predictors
        updateAllPoseTrackers();
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
                ? ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, rotate, EaseOfUse.generateRot2d( getHeading() ))
                : new ChassisSpeeds(driveX, driveY, rotate));
        
        // Limits the wheel speeds and sets the desired states
        setModuleStates(swerveModuleStates);
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

    /****************************************************************************************** 
    *
    *    UPDATER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * updateAllPoseTrackers()
     * <p>Updates all pose trackers
     */
    public void updateAllPoseTrackers() {
        updateOdometry();
        updatePoseEstimator();
    }

    /**
     * updateOdometry()
     * <p>Updates the odometry
     */
    private void updateOdometry() {
        // Compiles all the module positions
        SwerveModulePosition[] allModulePosition = getModulePositons();

        // Updates odometry
        odometry.update(
            EaseOfUse.generateRot2d( getHeading() ),
            allModulePosition);
    }

    /**
     * updatePoseEstimator()
     * <p>Updates the poseEstimator
     */
    private void updatePoseEstimator() {
        // Compiles all the module positions
        SwerveModulePosition[] allModulePosition = getModulePositons();

        // Updates the pose estimator
        poseEstimator.update(
            EaseOfUse.generateRot2d( getHeading() ),
            allModulePosition);

        // Gets current values
        Pose3d detPose = CustomTables.getBestResult();
        double detTime = CustomTables.getDetectionTime();

        // Adds the vision measurement if it hasn't been calculated yet
        if ((prevTime != detTime) && (prevPose != detPose)) {
            // Sets the prev variables
            prevTime = detTime;
            prevPose = detPose;

            // Adds the vision measurement
            poseEstimator.addVisionMeasurement(
                detPose.toPose2d(),
                detTime);
        }
    }

    /****************************************************************************************** 
    *
    *    RESETER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Resets both pose estimators to a specific pose
     * @param pose The pose to which to set the estimators.
     */
    public void resetPoseTrackers(Pose2d pose) {
        resetOdometry(pose);
        resetPoseEstimator(pose);
    }

    /**
     * Resets the odometry to the specified pose.
     * 
     * @param pose The pose to which to set the odometry.
     */
    private void resetOdometry(Pose2d pose) {
        odometry.resetPosition(EaseOfUse.generateRot2d( getHeading() ), getModulePositons(), pose);
    }

    /**
     * Resets the estimator to the specified pose.
     * 
     * @param pose The pose to which to set the estimator.
     */
    private void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(EaseOfUse.generateRot2d( getHeading() ), getModulePositons(), pose);
    }

    /****************************************************************************************** 
    *
    *    GETTER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Returns all the positions of each SwerveModule.
     * 
     * @return allPositions
     */
    public SwerveModulePosition[] getModulePositons() {
        SwerveModulePosition[] allPos = {
            frontLeftWheel.getPosition(),
            frontRightWheel.getPosition(),
            rearLeftWheel.getPosition(),
            rearRightWheel.getPosition()
        };

        return allPos;
    }

    /**
     * Returns the currently estimated pose of the robot, independant of the vision systems.
     *
     * @return Pose2d The estimated pose.
     */
    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the currently estimated pose of the robot, independant of the vision systems.
     *
     * @return Pose2d The estimated pose.
     */
    public Pose2d getVisionPose() {
        return poseEstimator.getEstimatedPosition();
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
        return ahrs.getRate();
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