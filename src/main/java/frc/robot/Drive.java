package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;

/**
 * Start of the Drive class
 */
public class Drive {
    // Constants
    private final Translation2d FRONT_LEFT_LOCATION;
    private final Translation2d FRONT_RIGHT_LOCATION;
    private final Translation2d BACK_LEFT_LOCATION;
    private final Translation2d BACK_RIGHT_LOCATION;
    private static final double MAX_DRIVE_SPEED   = 3; // Meters per second
    private static final double MAX_ROTATE_SPEED  = 2 * Math.PI; // Radians per second

    // Object Creation
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    public  SwerveDriveKinematics swerveDriveKinematics;

    // NavX
    private AHRS ahrs;

    /**
     * The constructor for the Drive class
     */
    public Drive() {
        // Defines the wheel translations (in meters)
        FRONT_LEFT_LOCATION  = new Translation2d(0.26035, 0.26035);
        FRONT_RIGHT_LOCATION = new Translation2d(0.26035, -0.26035);
        BACK_LEFT_LOCATION   = new Translation2d(-0.26035, 0.26035);
        BACK_RIGHT_LOCATION  = new Translation2d(-0.26035, -0.26035);

        // Creates the kinematics
        swerveDriveKinematics = new SwerveDriveKinematics(
            FRONT_LEFT_LOCATION,
            FRONT_RIGHT_LOCATION,
            BACK_LEFT_LOCATION,
            BACK_RIGHT_LOCATION
        );

        // Creates the swerve modules
        frontLeft  = new SwerveModule(16, 17, true);
        frontRight = new SwerveModule(10, 11, false);
        backLeft   = new SwerveModule(14, 15, true);
        backRight  = new SwerveModule(12, 13, false);

        // NavX creation
        try {
            ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            System.out.println("Error Instantiating NavX MXP: " + ex.getMessage());
        }
    
        ahrs.reset();
    
        while (ahrs.isConnected() == false) {
            // System.out.println("Connecting navX");
        }
        System.out.println("NavX Connected");
    
        while (ahrs.isCalibrating() == true) {
            System.out.println("Calibrating navX");
        }
        System.out.println("NavX Ready");
    
        ahrs.zeroYaw();
    }

    /****************************************************************************************** 
    *
    *    DRIVE FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * The function to drive the robot using a joystick.
     * 
     * @param forwardSpeed
     * @param strafeSpeed
     * @param rotationSpeed
     */
    public void teleopDrive(double forwardSpeed, double strafeSpeed, double rotationSpeed, boolean fieldDrive) {
        // Creates SwerveModuleStates from the drive powers
        SwerveModuleState[] swerveModuleStates = 
            swerveDriveKinematics.toSwerveModuleStates(
                fieldDrive
                ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, strafeSpeed, rotationSpeed, new Rotation2d( getHeading() ))
                : new ChassisSpeeds(forwardSpeed, strafeSpeed, rotationSpeed));

        // Limits the max drive speed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_DRIVE_SPEED);

        // Sets the desired module states
        frontLeft .move(swerveModuleStates[0]);
        frontRight.move(swerveModuleStates[1]);
        backLeft  .move(swerveModuleStates[2]);
        backRight .move(swerveModuleStates[3]);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Limits the wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_DRIVE_SPEED);

        //  Sets the desired states
        frontLeft .move(desiredStates[0]);
        frontRight.move(desiredStates[1]);
        backLeft  .move(desiredStates[2]);
        backRight .move(desiredStates[3]);
    }


    /****************************************************************************************** 
    *
    *    GETTER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Gets the Front Left Module's position.
     * 
     * @return FLPosition
     */
    public SwerveModulePosition getFLPosition() {
        return frontLeft.getModulePosition();
    }

    /**
     * Gets the Front Right Module's position.
     * 
     * @return FRPosition
     */
    public SwerveModulePosition getFRPosition() {
        return frontRight.getModulePosition();
    }

    /**
     * Gets the Back Left Module's position.
     * 
     * @return BLPosition
     */
    public SwerveModulePosition getBLPosition() {
        return backLeft.getModulePosition();
    }

    /**
     * Gets the Back Right Module's position.
     * 
     * @return BRPosition
     */
    public SwerveModulePosition getBRPosition() {
        return backRight.getModulePosition();
    }

    /***
     * Returns the robot's heading.
     * <p>This value has transformed to radians and negated to keep with convention.
     * 
     * @return headingRadians
     */
    public double getHeading() {
        return -1 * Math.PI * ahrs.getYaw();
    }

    /**
     * Gets the maximum drive speed
     * 
     * @return maxDriveSpeed
     */
    public static double getMaxDriveSpeed() {
        return MAX_DRIVE_SPEED;
    }

    /**
     * Gets the maximum rotation speed
     * 
     * @return maxRotateSpeed
     */
    public static double getMaxRotateSpeed() {
        return MAX_ROTATE_SPEED;
    }


    /****************************************************************************************** 
    *
    *    TEST FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Inits the motor sliders
     */
    public void initWheelPowerTests() {
        frontLeft.initMotorSliders();
        frontRight.initMotorSliders();
        backLeft.initMotorSliders();
        backRight.initMotorSliders();
    }

    /**
     * Tests the wheel powers
     */
    public void testWheelPowers() {
        frontLeft.updateMotorPowers();
        frontRight.updateMotorPowers();
        backLeft.updateMotorPowers();
        backRight.updateMotorPowers();
    }

    /**
     * Displays the enocder values
     */
    public void testEncoders() {
        frontLeft.displayEncoderValues();
        frontRight.displayEncoderValues();
        backLeft.displayEncoderValues();
        backRight.displayEncoderValues();
    }

    /**
     * Tests the modules by rotating them to different positions
     * 
     * @param radians
     */
    public void testModuleRotation(double radians) {
        // Creates the target positions
        SwerveModuleState   targetState      = new SwerveModuleState(0, new Rotation2d(radians));
        SwerveModuleState[] targetStateArray = {targetState, targetState, targetState, targetState};

        // Updates the encoders
        testEncoders();

        // Forces the wheels to move to it
        setModuleStates(targetStateArray);

        // Updates the encoders
        testEncoders();
    }
}

// End of the Drive class