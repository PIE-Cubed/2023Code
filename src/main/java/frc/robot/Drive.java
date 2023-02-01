package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Start of the Drive class
 */
public class Drive {
    // Constants
    private final double        MAX_TELEOP_SPEED = 1; // meters per second
    private final Translation2d FRONT_LEFT_LOCATION ;
    private final Translation2d FRONT_RIGHT_LOCATION;
    private final Translation2d BACK_LEFT_LOCATION  ;
    private final Translation2d BACK_RIGHT_LOCATION ;

    // Object Creation
    public static AHRS ahrs;
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    public SwerveDriveKinematics swerveDriveKinematics;

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
            System.out.println("Error Instantiating navX MXP: " + ex.getMessage());
        }
    
        ahrs.reset();
    
        while (ahrs.isConnected() == false) {
            System.out.println("Connecting navX");
        }
        System.out.println("navX Connected");
    
        while (ahrs.isCalibrating() == true) {
            System.out.println("Calibrating navX");
        }
        System.out.println("navx Ready");
    
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
     * @param forward
     * @param strafe
     * @param rotationSpeed
     */
    public void teleopDrive(double forward, double strafe, double rotationSpeed) {
        // Creates SwerveModuleStates from the drive powers
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(forward, strafe, rotationSpeed));

        // Limits the max drive speed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_TELEOP_SPEED);

        // Sets the desired module states
        frontLeft.move(swerveModuleStates[0]);
        frontRight.move(swerveModuleStates[1]);
        backLeft.move(swerveModuleStates[2]);
        backRight.move(swerveModuleStates[3]);
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
     * Returns the gyro's heading.
     * <p>This value has already been negated to keep with convention.
     * 
     * @return headingDegres
     */
    public double getHeading() {
        return ahrs.getYaw();
    }


    /****************************************************************************************** 
    *
    *    TEST FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Inits the motor sliders
     */
    public void initTestWheelPowers() {
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
}

// End of the Drive class