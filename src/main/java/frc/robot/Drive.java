package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
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

    public  static final double MAX_DRIVE_SPEED      = 4 * 1; // Meters per second - velocity is generally 4x the power
    public  static final double MAX_ROTATE_SPEED     = 4 * Math.PI; // Radians per second
    private static final double MAX_WHEEL_SPEED      = 4 * 1; // Meters per second

    private final double AUTO_DRIVE_TOLERANCE        = 0.05; //0.01
    private final double AUTO_DRIVE_ROTATE_TOLERANCE = 0.05; //0.15
    private final double RAMP_BALANCE_TOLERANCE      = 6;

    // Instance Variables
    private int     printCount         = 0;
    private int     autoPointIndex     = 0;
    private boolean autoPointFirstTime = true;
    private boolean autoPointAngled    = false; // Tracks if wheels have been angled before driving
    private boolean rampFirstTime      = true;  // Used by all ramp-related functions b/c only 1 called at a time
    private double  rampInitRoll       = 0;
    private double  initXVelocity      = 0;
    private double  initYVelocity      = 0;
    private double  initRotateVelocity = 0;
    private int     rampStep           = 1;
    private double  yawAdjustment      = Math.PI;

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
    private static final double adrp = MAX_ROTATE_SPEED * ((1.4) / Math.PI); // 1/1.4 Pi radians away --> full power
    private static final double adri = 0; //adrp / 50;
    private static final double adrd = 0;
    PIDController autoDriveRotateController;
    
    // Auto smooth drive to points Hyp controller
    private static final double asdp = MAX_WHEEL_SPEED; // 1 meter away --> full power
    private static final double asdi = 0.0;
    private static final double asdd = 0;
    PIDController autoSmoothDriveController;

    // Ramp balance controller
    private static final double rbP = -0.06;
    private static final double rbI = 0.00;
    private static final double rbD = 0.00;
    PIDController rampBalanceController;

    // Object Creation
    private SwerveModule frontLeft;
    private SwerveModule frontRight;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    public  SwerveDriveKinematics swerveDriveKinematics;

    // NavX
    public static AHRS ahrs;

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
         * whereas positive y values represent moving toward the left of the robot 
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

        // PID instantiation
        autoDriveXController = new PIDController(adp, adi, add);
        autoDriveXController.setTolerance(AUTO_DRIVE_TOLERANCE);

        autoDriveYController = new PIDController(adp, adi, add);
        autoDriveYController.setTolerance(AUTO_DRIVE_TOLERANCE);

        autoDriveRotateController = new PIDController(adrp, adri, adrd);
        autoDriveRotateController.setTolerance(AUTO_DRIVE_ROTATE_TOLERANCE);
        autoDriveRotateController.enableContinuousInput(Math.PI, -Math.PI);
        
        autoSmoothDriveController = new PIDController(asdp, asdi, asdd);
        autoSmoothDriveController.setTolerance(AUTO_DRIVE_TOLERANCE);
        autoSmoothDriveController.setSetpoint(0);

        rampBalanceController = new PIDController(rbP, rbI, rbD);
        rampBalanceController.setTolerance(RAMP_BALANCE_TOLERANCE);
    }

    /**
     * The function to drive the robot using a joystick.
     * <p>Positive Forward Goes Forward, Positive Strafe Goes Left, and Positive Rotation Speed is Counter-Clockwise 
     * @param forwardSpeed
     * @param strafeSpeed
     * @param rotationSpeed
     * @param fieldDrive
     */
    public void teleopDrive(double forwardSpeed, double strafeSpeed, double rotationSpeed, boolean fieldDrive) {
        // Calulates the SwerveModuleStates and determines if they are field relative
        SwerveModuleState[] swerveModuleStates = 
            swerveDriveKinematics.toSwerveModuleStates(
                fieldDrive
                ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, strafeSpeed, rotationSpeed, new Rotation2d( getYawAdjusted() ))
                : new ChassisSpeeds(forwardSpeed, strafeSpeed, rotationSpeed));

        // Limits the max speed of the wheels
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_WHEEL_SPEED);

        // Only moves wheels when given command
        if (forwardSpeed != 0 || strafeSpeed != 0 || rotationSpeed != 0) {
            // The SwerveModuleStates array index used must match the order from the SwerveDriveKinematics instantiation
            frontLeft.setDesiredState(swerveModuleStates[0]);
            frontRight.setDesiredState(swerveModuleStates[1]);
            backLeft.setDesiredState(swerveModuleStates[2]);
            backRight.setDesiredState(swerveModuleStates[3]);
        }
        else {
            stopWheels();
        }
    }

    /**
     * Drives wheels with direct power (-1 to 1), rather than velocity
     * @param forwardSpeed
     * @param strafeSpeed
     * @param rotationSpeed
     */
    public void testDrive(double forwardSpeed, double strafeSpeed, double rotationSpeed) {
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotationSpeed));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 1);
        frontLeft.directMove(swerveModuleStates[0]);
        frontRight.directMove(swerveModuleStates[1]);
        backLeft.directMove(swerveModuleStates[2]);
        backRight.directMove(swerveModuleStates[3]);
    }

    // AP: Consider moving some of these methods to another class. They make Drive absurdly long
    /**
     * Automatically drives through a list of points.
     * @param listOfPoints
     * @param currPose
     * @return
     */
    public int autoDriveToPoints(Pose2d[] listOfPoints, Pose2d currPose) {
        // Grabs the target point
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
            initRotateVelocity = autoDriveRotateController.calculate(getYawAdjusted(), targetPoint.getRotation().getRadians());
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
                double targetRotateVelocity = autoDriveRotateController.calculate(getYawAdjusted(), targetPoint.getRotation().getRadians());

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
            stopWheels();
            return Robot.DONE;
        }

        return Robot.CONT;
    }
    
    /*
     * Test version of autoDriveToPoints that should move more quickly
     * Uses hypotenuse of distance, rather than separate X and Y controllers
     */
    public int autoSmoothDriveToPoints(Translation2d[] listOfPoints, double endRadians, Pose2d currPose) {
        // Grabs the target point
        Translation2d targetPoint = listOfPoints[autoPointIndex];

        // This runs once for each point in the list
        if (autoPointFirstTime == true) {
            autoPointFirstTime = false;
            autoSmoothDriveController.reset();
            autoDriveRotateController.reset();
            autoDriveRotateController.setSetpoint(endRadians);

            autoPointAngled = false;
        } 
        // Runs when it's not the first time for a point
        else {
            // Calculating targetVelocity based on distance to targetPoint
            double targetVelocity = 0;
            double xError = currPose.getX() - listOfPoints[listOfPoints.length-1].getX();
            double yError = currPose.getY() - listOfPoints[listOfPoints.length-1].getY();
            if (autoPointIndex == listOfPoints.length - 1) {
                targetVelocity = autoSmoothDriveController.calculate(Math.hypot(xError, yError));
            }
            else {
                targetVelocity = MAX_WHEEL_SPEED * 0.8;
            }
            double driveAngle = Math.atan2(yError, xError);
            
            double targetRotateVelocity = autoDriveRotateController.calculate(getYawAdjusted(), endRadians);

            double targetXVelocity = Math.cos(driveAngle) * targetVelocity;
            double targetYVelocity = Math.sin(driveAngle) * targetVelocity;

            // Actual movement
            teleopDrive(targetXVelocity, targetYVelocity, targetRotateVelocity, true);
        }

        // If X, Y, and Rotation are at target, moves on to next point
        if (autoSmoothDriveController.atSetpoint() && autoDriveRotateController.atSetpoint()) {
            autoPointIndex++;
            autoPointFirstTime = true;
        }

        // Function ends once we pass the last point
        if (autoPointIndex >= listOfPoints.length) {
            autoPointIndex = 0;
            autoPointFirstTime = true;
            stopWheels();
            return Robot.DONE;
        }

        return Robot.CONT;
    }

    /*
     * Resets all instance variables used in driveToPoints
     */
    public void resetDriveToPoints() {
        autoPointFirstTime = true;
        autoPointIndex = 0;
    }

    /**
     * Drives toward ramp until it has been pressed down
     * @param frontEndFirst
     * @return
     */
    public int chargeRamp(boolean frontEndFirst) {
        int status = Robot.CONT;

        double changeInRoll;
        double targetRoll;

        if (rampFirstTime == true) {
            rampFirstTime = false;
            rampStep = 1;
            rampInitRoll = ahrs.getRoll();
        }

        switch(rampStep) {
            // Step 1: charge ramp until we go up by 20 degrees
            case 1:
                // Roll should increase by 20 if front end up, decrease by 20 if back end up
                changeInRoll = -20;
                if (frontEndFirst) {
                    changeInRoll *= -1;
                }
                targetRoll = rampInitRoll + changeInRoll;

                // If frontEndFirst, increasing roll should pass above target
                // If backEndFirst, decreasing roll should pass below target
                if (frontEndFirst) {
                    //System.out.println("Step 1 Current roll:" + ahrs.getRoll() + " target:>" + targetRoll);
                    if (ahrs.getRoll() > targetRoll) {
                        status = Robot.DONE;
                    }
                    else {
                        teleopDrive(3, 0, 0, false);
                        status = Robot.CONT;  
                    }
                }
                else {
                    //System.out.println("Step 1 Current roll:" + ahrs.getRoll() + " target:<" + targetRoll);
                    if (ahrs.getRoll() < targetRoll) {
                        status = Robot.DONE;
                    }
                    else {
                        teleopDrive(-3, 0, 0, false);
                        status = Robot.CONT;  
                    }
                }
                break;
            // Step 2: keep charging ramp until our angle comes back below 15
            case 2:
                // Roll should decrease from 20 to 15 if front end first, increase from -20 to -15 if back end first
                changeInRoll = -15;
                if (frontEndFirst) {
                    changeInRoll *= -1;
                }
                targetRoll = rampInitRoll + changeInRoll;

                // If frontEndFirst, roll should start decreasing and pass below target
                // If backEndFirst, roll should start increasing and pass above target
                if (frontEndFirst) {
                    //System.out.println("Step 2 Current roll:" + ahrs.getRoll() + " target:<" + targetRoll);
                    if (ahrs.getRoll() < targetRoll) {
                        status = Robot.DONE;
                    }
                    else {
                        teleopDrive(3, 0, 0, false);
                        status = Robot.CONT;  
                    }
                }
                else {
                    if (ahrs.getRoll() > targetRoll) {
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

    /**
     * Drives off ramp until it has been pressed down
     * @param frontEndFirst
     * @return
     */
    public int leaveRamp(boolean frontEndFirst) {
        if (rampFirstTime) {
            rampFirstTime = false;
            rampInitRoll = ahrs.getRoll();
        }

        // Roll should decrease by 12 if front end up, increase by 12 if back end up (opposite of going up ramp)
        double changeInRoll = -12;
        if (frontEndFirst == false) {
            changeInRoll *= -1;
        }
        double targetRoll = rampInitRoll + changeInRoll;

        // If frontEndFirst, decreasing roll should pass below target
        // If backEndFirst, increasing roll should pass above target
        if (frontEndFirst) {
            if (ahrs.getRoll() < targetRoll) {
                rampFirstTime = true;
                return Robot.DONE;
            }
            else {
                teleopDrive(3, 0, 0, false);
                return Robot.CONT;  
            }
        }
        else {
            if (ahrs.getRoll() > targetRoll) {
                rampFirstTime = true;
                return Robot.DONE;
            }
            else {
                teleopDrive(-3, 0, 0, false);
                return Robot.CONT;  
            }
        }
    }

    /**
     * Drives forward until ramp levels, then immediately stops
     * @param targetRoll
     * @return
     */
    public int balanceRamp(double targetRoll) {
        // Calculating targetVelocity based on distance to targetPoint
        rampBalanceController.setSetpoint(targetRoll);

        double driveVelocity = rampBalanceController.calculate(ahrs.getRoll(), targetRoll);
        driveVelocity = MathUtil.clamp(driveVelocity, -1.5, 1.5);

        // Does movement until routine ends
        if (rampBalanceController.atSetpoint() == true) {
            crossWheels();
            return Robot.DONE;
        }
        else {
            teleopDrive(driveVelocity, 0, 0, false);
        }

        return Robot.CONT;
    }

    /**
     * Rotates wheels based on a drive command without giving the drive motors full power
     * @param driveX
     * @param driveY
     * @param driveZ
     * @param fieldDrive
     * @return
     */
    public int rotateWheels(double driveX, double driveY, double driveZ, boolean fieldDrive) {
        teleopDrive(driveX / 100, driveY / 100, driveZ / 100, fieldDrive);

        if (frontLeft.rotateControllerAtSetpoint() && frontRight.rotateControllerAtSetpoint() &&
            backLeft.rotateControllerAtSetpoint() && backRight.rotateControllerAtSetpoint()) {
                return Robot.DONE;
        }

        return Robot.CONT;
    }


    /****************************************************************************************** 
    *
    *    SETTING FUNCTIONS
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
    *    HELPER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Stops the wheels.
     */
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

    /**
     * Crosses the wheels and makes the robot impossible to move.
     */
    public void crossWheels() {
        frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d( Math.PI / 4 )));
        frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d( -Math.PI / 4 )));
        backLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d( -Math.PI / 4 )));
        backRight.setDesiredState(new SwerveModuleState(0, new Rotation2d( Math.PI / 4 )));       
    }

    /**
     * Resets the Yaw on the NavX.
     */
    public void resetYaw() {
        ahrs.zeroYaw();
    }

    public boolean gyroConnected() {
        return ahrs.isConnected();
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

    /*
     * Adjusts for all autos starting facing backwards
     */
    public double getYawAdjusted() {
        return MathUtil.angleModulus(-1 * Units.degreesToRadians( ahrs.getYaw() ) + yawAdjustment);
    }

    public void correctYaw(double radians) {
        yawAdjustment += radians;
    }

    /**
     * Gets the roll from the NavX.
     * 
     * @return robotRoll
     */
    public double getRoll() {
        return ahrs.getRoll();
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

    /**
     * 
     */
    public void printPowerandVelocity() {
        if (printCount % 15 == 0) {
            frontLeft.displayPowerAndVelocity();
            frontRight.displayPowerAndVelocity();
            backLeft.displayPowerAndVelocity();
            backRight.displayPowerAndVelocity();
        }
        printCount++;
    }

    /**
     * 
     */
    public void testGyro() {
        if (printCount % 15 == 0) {
            System.out.println("Adjusted angle: " + getYawAdjusted());
        }
        printCount++;
    }

    /**
     * 
     */
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
