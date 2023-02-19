package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Start of the Drive class
 */
public class Drive {
    // Constants
    private final Translation2d FRONT_LEFT_LOCATION;
    private final Translation2d FRONT_RIGHT_LOCATION;
    private final Translation2d BACK_LEFT_LOCATION;
    private final Translation2d BACK_RIGHT_LOCATION;

    public  static final double MAX_DRIVE_SPEED      = 4; // Meters per second - velocity is generally 4x the power
    public  static final double MAX_ROTATE_SPEED     = 4 * Math.PI; // Radians per second
    private static final double MAX_WHEEL_SPEED      = 4; // Meters per second

    private final double AUTO_DRIVE_TOLERANCE        = 0.05; //0.01
    private final double AUTO_DRIVE_ROTATE_TOLERANCE = 0.15; //0.075
    private final double RAMP_BALANCE_TOLERANCE      = 2;

    // Instance Variables
    private int     printCount             = 0;
    private int     autoPointIndex         = 0;
    private boolean autoPointFirstTime     = true;
    private boolean autoPointAngled        = false; // Tracks if wheels have been angled before driving
    private boolean rampFirstTime          = true; // Used by all ramp-related functions b/c only 1 called at a time
    private double  rampInitPitch          = 0;
    private double  initXVelocity          = 0;
    private double  initYVelocity          = 0;
    private double  initRotateVelocity     = 0;
    private int     rampStep               = 1;

    // NAVX
    public static AHRS ahrs;

    // Rate limiters for auto drive
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter rotateLimiter;
    
    // Auto drive to points X controller - need 2 controllers for X and Y for both setpoints
    private static final double adp = MAX_WHEEL_SPEED; // 1 meter away --> full power
    private static final double adi = 0.0;
    private static final double add = 0;
    PIDController autoDriveXController;
    PIDController autoDriveYController;

    // Auto drive to points rotate controller
    private static final double adrp = MAX_ROTATE_SPEED * (1 / (2 * Math.PI)) ; // 2*Pi radians away --> full power
    private static final double adri = 0.0;
    private static final double adrd = 0;
    PIDController autoDriveRotateController;

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

    // Ramp balance controller
    private static final double rbP = 0.06; //-0.04 works, but slow
    private static final double rbI = 0.00;
    private static final double rbD = 0.00;
    PIDController rampBalanceController;

    // Object Creation
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    public  SwerveDriveKinematics swerveDriveKinematics;

    /**
     * The constructor for the Drive class
     */
    public Drive() {
        // NavX
        try {
            ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            System.out.println("Error Instantiating navX MXP: " + ex.getMessage());
        }

        ahrs.reset();

        while (ahrs.isConnected() == false) {
            // System.out.println("Connecting navX");
        }
        System.out.println("navX Connected");

        while (ahrs.isCalibrating() == true) {
            System.out.println("Calibrating navX");
        }
        System.out.println("navX Ready");

        ahrs.zeroYaw();

        // Initializing rate limiters
        xLimiter = new SlewRateLimiter(24);
        yLimiter = new SlewRateLimiter(24);
        rotateLimiter = new SlewRateLimiter(8 * Math.PI);

        /* The locations for the modules must be relative to the center of the robot. 
         * Positive x values represent moving toward the front of the robot 
         *  whereas positive y values represent moving toward the left of the robot 
         * Values are in meters
        */
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

        autoDriveXController = new PIDController(adp, adi, add);
        autoDriveXController.setTolerance(AUTO_DRIVE_TOLERANCE);

        autoDriveYController = new PIDController(adp, adi, add);
        autoDriveYController.setTolerance(AUTO_DRIVE_TOLERANCE);

        autoDriveRotateController = new PIDController(adrp, adri, adrd);
        autoDriveRotateController.setTolerance(AUTO_DRIVE_ROTATE_TOLERANCE);
        autoDriveRotateController.enableContinuousInput(Math.PI, -Math.PI);

        rampBalanceController = new PIDController(rbP, rbI, rbD);
        rampBalanceController.setTolerance(RAMP_BALANCE_TOLERANCE);

        // Used during crab drive to keep the robot at same orientation
        autoCrabDriveController = new PIDController(acdP, acdI, acdD);
        autoCrabDriveController.enableContinuousInput(-180.0, 180.0);
        autoCrabDriveController.setTolerance(2);

        autoRotateController = new PIDController(arP, arI, arD);
        autoRotateController.enableContinuousInput(-180.0, 180.0);
        autoRotateController.setTolerance(2);
    }

    /**
     * The function to drive the robot using a joystick.
     * Positive Forward Goes Forward
     * Positive Strafe Goes Left
     * Positive Rotation Speed is Counter-Clockwise 
     * @param forwardSpeed
     * @param strafeSpeed
     * @param rotationSpeed
     * @param fieldOriented
     */
    public void teleopDrive(double forward, double strafe, double rotationSpeed, boolean fieldOriented) {
        SwerveModuleState[] swerveModuleStates; 

        if (fieldOriented) {
            try {
                swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotationSpeed, ( new Rotation2d( getZ() ))));
            } 
            catch (Exception e) {
                swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(forward, strafe, rotationSpeed));
            }
        }
        else {
            swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(forward, strafe, rotationSpeed));
        }
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_WHEEL_SPEED);

        /* The swerveModuleStates array index used must match the order from the SwerveDriveKinematics instantiation */
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    // Directly feeds joystick values into motors for testing
    public void testDrive(double forward, double strafe, double rotationSpeed) {
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(forward, strafe, rotationSpeed));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 1);
        frontLeft.directMove(swerveModuleStates[0]);
        frontRight.directMove(swerveModuleStates[1]);
        backLeft.directMove(swerveModuleStates[2]);
        backRight.directMove(swerveModuleStates[3]);
    }

    /**
     * 
     * @param listOfPoints
     * @return
     */
    public int autoDriveToPoints(Pose2d[] listOfPoints, Pose2d currPose) {
        Pose2d targetPoint = listOfPoints[autoPointIndex];

        // This runs once for each point in the list
        if (autoPointFirstTime == true) {
            autoPointFirstTime = false;
            autoDriveXController.reset();
            autoDriveYController.reset();
            autoDriveRotateController.reset();
            autoDriveXController.setSetpoint(targetPoint.getX());
            autoDriveYController.setSetpoint(targetPoint.getY());
            autoDriveRotateController.setSetpoint(targetPoint.getRotation().getRadians());

            autoPointAngled = false;

            System.out.println("X target:" + targetPoint.getX() + " Y target:" + targetPoint.getY() + " Z target:" + targetPoint.getRotation().getRadians());

            // For each point except the last
            if (autoPointIndex < listOfPoints.length - 1) {
                autoDriveXController.setTolerance(2 * AUTO_DRIVE_TOLERANCE);
                autoDriveYController.setTolerance(2 * AUTO_DRIVE_TOLERANCE);
                autoDriveRotateController.setTolerance(2 * AUTO_DRIVE_ROTATE_TOLERANCE);
            }
            else {
                autoDriveXController.setTolerance(AUTO_DRIVE_TOLERANCE);
                autoDriveYController.setTolerance(AUTO_DRIVE_TOLERANCE);
                autoDriveRotateController.setTolerance(AUTO_DRIVE_ROTATE_TOLERANCE);
            }

            initXVelocity      = autoDriveXController.calculate(currPose.getX(), targetPoint.getX());
            initYVelocity      = autoDriveYController.calculate(currPose.getY(), targetPoint.getY());
            initRotateVelocity = autoDriveRotateController.calculate(getZ(), targetPoint.getRotation().getRadians());
        } 
        // Runs when it's not the first time for a point
        else {
            // Angles the wheels if they are not aligned before driving
            if (autoPointAngled == false) {
                int rotateStatus = rotateWheels(initXVelocity, initYVelocity, initRotateVelocity, true);
                if (rotateStatus == Robot.DONE) {
                    autoPointAngled = true;
                }
            }
            // Drives normally once wheels are angled
            else {
                // Calculating targetVelocity based on distance to targetPoint
                double targetXVelocity      = autoDriveXController.calculate(currPose.getX(), targetPoint.getX());
                double targetYVelocity      = autoDriveYController.calculate(currPose.getY(), targetPoint.getY());
                double targetRotateVelocity = autoDriveRotateController.calculate(getZ(), targetPoint.getRotation().getRadians());

                targetXVelocity = xLimiter.calculate(targetXVelocity);
                targetYVelocity = yLimiter.calculate(targetYVelocity);
                targetRotateVelocity = rotateLimiter.calculate(targetRotateVelocity);


                // Actual movement -  only if wheels are rotated
                teleopDrive(targetXVelocity, targetYVelocity, targetRotateVelocity, true);
            }
        }

        // If X, Y, and Rotation are at target, moves on to next point
        if (autoDriveXController.atSetpoint() && autoDriveYController.atSetpoint() && autoDriveRotateController.atSetpoint()) {
            autoPointIndex++;
            autoPointFirstTime = true;
        }

        // Function ends once we pass the last point
        if (autoPointIndex >= listOfPoints.length) {
            autoPointIndex = 0;
            autoPointFirstTime = true;
            return Robot.DONE;
        }

        return Robot.CONT;
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

    // Should be used to apply stronger brakes
    public void crossWheels() {
        frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.PI/4)));
        frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(-Math.PI/4)));
        backLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(-Math.PI/4)));
        backRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.PI/4)));       
    }

    public Rotation2d getYaw() {
        return ahrs.getRotation2d();
    }

    public double getPitch() {
        return ahrs.getPitch();
    }

    public void resetYaw() {
        // Resets odometry to link our current pose with the new gyro angle
        ahrs.zeroYaw();
    }

    public void setAngleAdjustment(double radians) {
        ahrs.setAngleAdjustment(radians);
    }

    public int chargeRamp(boolean frontEndFirst) {
        int status = Robot.CONT;

        double changeInPitch;
        double targetPitch;

        if (rampFirstTime) {
            rampFirstTime = false;
            rampStep = 1;
            rampInitPitch = ahrs.getPitch();
        }

        switch(rampStep) {
            // Step 1: charge ramp until we go up by 20 degrees
            case 1:
                // Pitch should decrease by 20 if front end up, increase by 20 if back end up
                changeInPitch = 20;
                if (frontEndFirst) {
                    changeInPitch *= -1;
                }
                targetPitch = rampInitPitch + changeInPitch;

                // If frontEndFirst, decreasing pitch should pass below target
                // If backEndFirst, increasing pitch should pass above target
                System.out.println("Current pitch:" + ahrs.getPitch() + " target:>" + targetPitch);
                if (frontEndFirst) {
                    if (ahrs.getPitch() < targetPitch) {
                        status = Robot.DONE;
                    }
                    else {
                        teleopDrive(3, 0, 0, false);
                        status = Robot.CONT;  
                    }
                }
                else {
                    if (ahrs.getPitch() > targetPitch) {
                        System.out.println("Status is DONE");
                        status = Robot.DONE;
                    }
                    else {
                        teleopDrive(-3, 0, 0, false);
                        status = Robot.CONT;  
                    }
                }
                break;
            // Step 2: keep charging ramp until our angle comes back below 8
            case 2:
                // Pitch should increase from -20 to -15 if front end first, decrease from 20 to 15 if back end first
                changeInPitch = 15;
                if (frontEndFirst) {
                    changeInPitch *= -1;
                }
                targetPitch = rampInitPitch + changeInPitch;

                // If frontEndFirst, pitch should start increasing and pass above target
                // If backEndFirst, pitch should start decreasing and pass below target
                System.out.println("Current pitch:" + ahrs.getPitch() + " target:<" + targetPitch);
                if (frontEndFirst) {
                    if (ahrs.getPitch() > targetPitch) {
                        status = Robot.DONE;
                    }
                    else {
                        teleopDrive(3, 0, 0, false);
                        status = Robot.CONT;  
                    }
                }
                else {
                    if (ahrs.getPitch() < targetPitch) {
                        status = Robot.DONE;
                    }
                    else {
                        teleopDrive(-3, 0, 0, false);
                        status = Robot.CONT;  
                    }
                }
                break;
            default:
                rampStep = 1;
                rampFirstTime = true;
                System.out.println("Returning done");
                return Robot.DONE;      
        }

        if (status == Robot.DONE) {
            rampStep++;
            System.out.println("Going to step " + rampStep);
        }
            
        return Robot.CONT; 
    }

    public int leaveRamp(boolean frontEndFirst) {
        if (rampFirstTime) {
            rampFirstTime = false;
            rampInitPitch = ahrs.getPitch();
        }

        // Pitch should increase by 12 if front end up, decrease by 12 if back end up (opposite of going up ramp)
        double changeInPitch = 12;
        if (frontEndFirst == false) {
            changeInPitch *= -1;
        }
        double targetPitch = rampInitPitch + changeInPitch;

        // If frontEndFirst, increasing pitch should pass above target
        // If backEndFirst, decreasing pitch should pass below target
        if (frontEndFirst) {
            if (ahrs.getPitch() > targetPitch) {
                rampFirstTime = true;
                return Robot.DONE;
            }
            else {
                teleopDrive(3, 0, 0, false);
                return Robot.CONT;  
            }
        }
        else {
            if (ahrs.getPitch() < targetPitch) {
                rampFirstTime = true;
                return Robot.DONE;
            }
            else {
                teleopDrive(-3, 0, 0, false);
                return Robot.CONT;  
            }
        }
    }

    public int balanceRamp(double targetPitch) {
        // Calculating targetVelocity based on distance to targetPoint
        rampBalanceController.setSetpoint(targetPitch);

        double driveVelocity = rampBalanceController.calculate(ahrs.getPitch(), targetPitch);
        driveVelocity = MathUtil.clamp(driveVelocity, -1.5, 1.5);

        // Does movement until routine ends
        if (rampBalanceController.atSetpoint()) {
            crossWheels();
            return Robot.DONE;
        }
        else {
            teleopDrive(driveVelocity, 0, 0, false);
        }

        return Robot.CONT;
    }

    public int rotateWheels(double driveX, double driveY, double driveZ, boolean fieldDrive) {
        teleopDrive(driveX / 100, driveY / 100, driveZ / 100, fieldDrive);

        if (frontLeft.rotateControllerAtSetpoint() && frontRight.rotateControllerAtSetpoint() &&
            backLeft.rotateControllerAtSetpoint() && backRight.rotateControllerAtSetpoint()) {
                return Robot.DONE;
        }

        return Robot.CONT;
    }

    public double getZ() {
        return MathUtil.angleModulus(Math.toRadians(ahrs.getAngle()));
    }

    public void setGyroAngleZero(double degrees) {
        ahrs.setAngleAdjustment(-degrees);
    }

    /****************************************************************************************** 
    *
    *    DRIVE FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Limits the wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_DRIVE_SPEED);

        // Sets the desired states
        frontLeft .setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft  .setDesiredState(desiredStates[2]);
        backRight .setDesiredState(desiredStates[3]);
    }


    /****************************************************************************************** 
    *
    *    GETTER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Gets the Front Left Module's position.
     * 
     * @return The FrontLeft Module Position
     */
    public SwerveModulePosition getFLPosition() {
        return frontLeft.getModulePosition();
    }

    /**
     * Gets the Front Right Module's position.
     * 
     * @return The FrontRight Module Position
     */
    public SwerveModulePosition getFRPosition() {
        return frontRight.getModulePosition();
    }

    /**
     * Gets the Back Left Module's position.
     * 
     * @return The BackLeft Module Position
     */
    public SwerveModulePosition getBLPosition() {
        return backLeft.getModulePosition();
    }

    /**
     * Gets the Back Right Module's position.
     * 
     * @return The BackRight Module Position
     */
    public SwerveModulePosition getBRPosition() {
        return backRight.getModulePosition();
    }

    /**
     * Returns the robot's heading.
     * <p>This value has transformed to radians and negated to keep with convention.
     * 
     * @return The robot's heading in radians
     */
    public double getHeading() {
        return -1 * Units.degreesToRadians( ahrs.getYaw() );
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
     * Zeros the motor encoders
     */
    public void zeroMotorEncoders() {
        frontLeft .zeroMotorEncoders();
        frontRight.zeroMotorEncoders();
        backLeft  .zeroMotorEncoders();
        backRight .zeroMotorEncoders();
    }

    /**
     * Displays the enocder values
     */
    public void testEncoders() {
        frontLeft .displayEncoderValues();
        frontRight.displayEncoderValues();
        backLeft  .displayEncoderValues();
        backRight .displayEncoderValues();
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

    public void printPowerandVelocity() {
        //frontLeft.displayPowerAndVelocity();
        //frontRight.displayPowerAndVelocity();
        //backLeft.displayPowerAndVelocity();
        //backRight.displayPowerAndVelocity();
    }

    public void testGyro() {
        if (printCount % 15 == 0) {
            //System.out.println(ahrs.getRotation2d().getDegrees());
            System.out.println("Yaw=" + String.format( "%.2f", ahrs.getYaw()) + " Roll=" + String.format( "%.2f", ahrs.getRoll()) + " Pitch=" + String.format( "%.2f", ahrs.getPitch()));
        }
        printCount++;
    }

    public void initTestDrivePower() {
        SmartDashboard.putNumber("Drive Power", 0);
    }

    public void periodicTestDrivePower() {
        double drivePower = SmartDashboard.getNumber("Drive Power", 0);
        testDrive(drivePower, 0, 0);

        if (printCount % 5 == 0) {
            System.out.println("Drive Speed: " + frontLeft.getDriveVelocity());
        }
        printCount++;
    }
}
// End of the Drive class