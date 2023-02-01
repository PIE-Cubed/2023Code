package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;


/**
 * Start of the Drive class
 */
public class Drive {
    // Object Creation
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    private SwerveDriveKinematics swerveDriveKinematics;

    // Constants for Wheel Distance, Rotation, and Size 
    private final double wheelDiameterInches = 3;
    private final double clicksPerWheelRotation = 5.5;
    private final double inchesPerWheelRotation = wheelDiameterInches * Math.PI;
    private final double clicksPerInches = clicksPerWheelRotation / inchesPerWheelRotation;
    private final double clicksPerFoot = clicksPerInches / 12;

    // Constants
    private final Translation2d FRONT_LEFT_LOCATION;
    private final Translation2d FRONT_RIGHT_LOCATION;
    private final Translation2d BACK_LEFT_LOCATION;
    private final Translation2d BACK_RIGHT_LOCATION;
    private final double        MAX_TELEOP_SPEED = 1;

    // Instance Variables
    private int rotateCount = 0;
    private double rotateTimeOut = 0;
    private boolean firstTimeAutoCrabDrive = true;
    private boolean firstTimeAutoRotate    = true;
    private double encoderTarget;
    
    // NAVX
    public static AHRS ahrs;
    
    // Auto crab drive controller
    private static final double acdP = 0.005; 
    private static final double acdI = 0.000;
    private static final double acdD = 0.000;
    PIDController autoCrabDriveController;

    // Auto rotate controller
    private static final double arP = 0.01; 
    private static final double arI = 0.000;
    private static final double arD = 0.000;
    PIDController autoRotateController;

    private double targetOrientation = 0;

    public Drive() {
        // NavX
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

        
        // Values are in meters
        FRONT_LEFT_LOCATION  = new Translation2d(0.26035, 0.26035);
        FRONT_RIGHT_LOCATION = new Translation2d(0.26035, -0.26035);
        BACK_LEFT_LOCATION   = new Translation2d(-0.26035, 0.26035);
        BACK_RIGHT_LOCATION  = new Translation2d(-0.26035, -0.26035);

        frontLeft  = new SwerveModule(16, 17, true);
        frontRight = new SwerveModule(10, 11, false);
        backLeft   = new SwerveModule(14, 15, true);
        backRight  = new SwerveModule(12, 13, false);

        swerveDriveKinematics = new SwerveDriveKinematics(FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);
        
        // Used during crab drive to keep the robot at same orientation
        autoCrabDriveController = new PIDController(acdP, acdI, acdD);
        autoCrabDriveController.enableContinuousInput(-180.0, 180.0);
        autoCrabDriveController.setTolerance(2);

        autoRotateController = new PIDController(arP, arI, arD);
        autoRotateController.enableContinuousInput(-180.0, 180.0);
        autoRotateController.setTolerance(2);
    }
    
    /*Positive Forward Goes Forward
     *Positive Strafe Goes Left
     *Positive Rotation Speed is Counter-Clockwise*/
    public void teleopDrive(double forward, double strafe, double rotationSpeed) {
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(forward, strafe, rotationSpeed));
        //SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_TELEOP_SPEED);
        frontLeft.move(swerveModuleStates[0]);
        frontRight.move(swerveModuleStates[1]);
        backLeft.move(swerveModuleStates[2]);
        backRight.move(swerveModuleStates[3]);
    }

    public int autoCrabDrive(double distanceFeet, double power, double targetHeading) { 
        double encoderCurrent = getAverageEncoder(); //Average of 4 wheels
        double rotatePower;


        // First time through initializes target values
        if (firstTimeAutoCrabDrive == true) {
            firstTimeAutoCrabDrive = false;
            targetOrientation = ahrs.getYaw();
            encoderTarget = encoderCurrent + (clicksPerFoot * distanceFeet);
        }
        
        double orientationError = 0; //Temporary, need to add gyro
        double forwardPower = power * Math.sin(Math.toRadians(targetHeading));
        double strafePower = power * Math.cos(Math.toRadians(targetHeading));
                
        rotatePower = autoCrabDriveController.calculate(ahrs.getYaw(), targetOrientation); 

        teleopDrive(forwardPower, strafePower, rotatePower); 

        // Checks if target distance has been reached, then ends function if so
        if (encoderCurrent >= encoderTarget) {
            firstTimeAutoCrabDrive = true;
           stopWheels();
           // rotateController.reset();
            return Robot.DONE;
        } 
        else {
            return Robot.CONT;
        }
    }

    public int autoRotate(double degrees) {
        double currentMS = System.currentTimeMillis();

        // First Time
        if (firstTimeAutoRotate) {
            firstTimeAutoRotate = false;
            rotateCount = 0;
            rotateTimeOut = currentMS + 2000;
        }

        // Checking for timeout
        if (currentMS > rotateTimeOut) {
            stopWheels();
            firstTimeAutoRotate = true;
            rotateCount = 0;
            return Robot.DONE;
        }

        // Rotating
        double rotatePower = autoRotateController.calculate(ahrs.getYaw(), degrees);
        teleopDrive(0, 0, rotatePower);
    
        // Checking if on set point
        if (autoRotateController.atSetpoint()) {
            rotateCount++;
        }
        else {
            rotateCount = 0;
        }

        if (rotateCount > 10) {
            stopWheels();
            firstTimeAutoRotate = true;
            rotateCount = 0;
            return Robot.DONE;
        }
        else {
            return Robot.CONT;
        }
    }


    public void stopWheels() {
        frontLeft.setDriveMotorPower(0.0);
        frontLeft.setRotateMotorPower(0.0);
        frontRight.setDriveMotorPower(0.0);
        frontRight.setRotateMotorPower(0.0);
        backLeft.setDriveMotorPower(0.0);
        backLeft.setRotateMotorPower(0.0);
        backRight.setDriveMotorPower(0.0);
        backRight.setRotateMotorPower(0.0);
    }


    private double getAverageEncoder() {
        return ( frontLeft.getDriveEncoder()  +
                 frontRight.getDriveEncoder() + 
                 backLeft.getDriveEncoder()   + 
                 backRight.getDriveEncoder() ) / 4;
    }

    

    /***********************************************************************************************
     *
     *      TEST CODE
     *
     ***********************************************************************************************/
    /**
     * 
     */
    public void testEncoders() {
        //frontLeft.displayEncoderValues();
        //frontRight.displayEncoderValues();
        backLeft.displayEncoderValues();
        //backRight.displayEncoderValues();
    }

    public void initTestWheelPower() {
        backLeft.initDriveMotorSlider();
        backLeft.initRotateMotorSlider();
    }
    
    public void testWheelPower() {
        backLeft.updateMotorPeriodic();
    }
}

// End of the Drive class