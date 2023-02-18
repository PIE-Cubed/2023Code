package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private SwerveDriveOdometry   swerveDriveOdometry;
    private Pose2d                pose;


    // CAB- these constants should be replaced with existing constants for meters
    private final double wheelDiameterInches = 3;
    private final double clicksPerWheelRotation = 5.5;
    private final double inchesPerWheelRotation = wheelDiameterInches * Math.PI;
    private final double clicksPerInches = clicksPerWheelRotation / inchesPerWheelRotation;
    private final double clicksPerFoot = clicksPerInches * 12;

    // Constants
    private final Translation2d FRONT_LEFT_LOCATION;
    private final Translation2d FRONT_RIGHT_LOCATION;
    private final Translation2d BACK_LEFT_LOCATION;
    private final Translation2d BACK_RIGHT_LOCATION;
    /*
     * TJM
     *  We should verify this to ensure we can acheive the max performance from the robot
     */
    private static final double MAX_TELEOP_SPEED     = 4; // Meters per second - velocity is generally 4x the power
    private static final double MAX_ROTATION_SPEED   = 4 * Math.PI; // Radians per second
    private static final double MAX_WHEEL_SPEED      = 4; // Meters per second

    private final double MAX_APRIL_TAG_ERROR         = 10;
    private final double AUTO_DRIVE_TOLERANCE        = 0.05; //0.01
    private final double AUTO_DRIVE_ROTATE_TOLERANCE = 0.15; //0.075
    private final double RAMP_BALANCE_TOLERANCE      = 2;

    // Instance Variables
    private int     rotateCount            = 0;
    private double  rotateTimeOut          = 0;
    private int     printCount             = 0;
    private boolean firstTimeAutoCrabDrive = true;
    private boolean firstTimeAutoRotate    = true;
    private double  encoderTarget;
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
    private static final double adrp = MAX_ROTATION_SPEED * (1 / (2 * Math.PI)) ; // 2*Pi radians away --> full power
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

        /*
         * Inversion values come from applying positive power to wheel
         * and verifying the wheel moves forward
         */
        frontLeft  = new SwerveModule(16, 17, true);
        frontRight = new SwerveModule(10, 11, false);
        backLeft   = new SwerveModule(14, 15, true);
        backRight  = new SwerveModule(12, 13, false);

        swerveDriveKinematics = new SwerveDriveKinematics(FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);
        swerveDriveOdometry   = new SwerveDriveOdometry(swerveDriveKinematics, ahrs.getRotation2d(), new SwerveModulePosition[]{ 
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
        });

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
    
    /*Positive Forward Goes Forward
     *Positive Strafe Goes Left
     *Positive Rotation Speed is Counter-Clockwise*/
    public void teleopDrive(double forward, double strafe, double rotationSpeed, boolean fieldOriented) {
        SwerveModuleState[] swerveModuleStates; 

        if (fieldOriented) {
            try {
                swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotationSpeed, ( ahrs.getRotation2d() )));
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
        frontLeft.move(swerveModuleStates[0]);
        frontRight.move(swerveModuleStates[1]);
        backLeft.move(swerveModuleStates[2]);
        backRight.move(swerveModuleStates[3]);
    }

    /*
     * TJM
     * How did this test go???
     */
    // Directly feeds joystick values into motors for testing
    public void testDrive(double forward, double strafe, double rotationSpeed) {
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(forward, strafe, rotationSpeed));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 1);
        frontLeft.directMove(swerveModuleStates[0]);
        frontRight.directMove(swerveModuleStates[1]);
        backLeft.directMove(swerveModuleStates[2]);
        backRight.directMove(swerveModuleStates[3]);
    }

    // Autonomous function to drive to inputted list of points
    public int autoDriveToPoints(Pose2d[] listOfPoints) {
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

            initXVelocity      = autoDriveXController.calculate(getX(), targetPoint.getX());
            initYVelocity      = autoDriveYController.calculate(getY(), targetPoint.getY());
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
                double targetXVelocity      = autoDriveXController.calculate(getX(), targetPoint.getX());
                double targetYVelocity      = autoDriveYController.calculate(getY(), targetPoint.getY());
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

    public int autoCrabDrive(double distanceFeet, double power, double targetHeading) { 
        // double encoderCurrent = getAverageEncoder(); //Average of 4 wheels
        double encoderCurrent = frontRight.getDriveEncoder(); //Average of 4 wheels

        double rotatePower;

        // First time through initializes target values
        if (firstTimeAutoCrabDrive == true) {
            firstTimeAutoCrabDrive = false;
            targetOrientation = ahrs.getYaw();
            encoderTarget = encoderCurrent + (clicksPerFoot * distanceFeet);
        }
        System.out.println("Encoder target: "+ encoderTarget + " Encoder current: " + encoderCurrent);

        double orientationError = 0; // Temporary, need to add gyro
        double forwardPower = power * Math.sin(Math.toRadians(targetHeading));
        double strafePower  = power * Math.cos(Math.toRadians(targetHeading));

        rotatePower = 0; // autoCrabDriveController.calculate(ahrs.getYaw(), targetOrientation);

        teleopDrive(forwardPower, strafePower, rotatePower, false); 

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
        teleopDrive(0, 0, rotatePower, false);
    
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

    // Should be used to apply stronger brakes
    public void crossWheels() {
        frontLeft.move(new SwerveModuleState(0, new Rotation2d(Math.PI/4)));
        frontRight.move(new SwerveModuleState(0, new Rotation2d(-Math.PI/4)));
        backLeft.move(new SwerveModuleState(0, new Rotation2d(-Math.PI/4)));
        backRight.move(new SwerveModuleState(0, new Rotation2d(Math.PI/4)));       
    }

    private double getAverageEncoder() {
        return ( frontLeft.getDriveEncoder()  +
                 frontRight.getDriveEncoder() + 
                 backLeft.getDriveEncoder()   + 
                 backRight.getDriveEncoder() ) / 4;
    }

    public static double getMaxSpeed() {
        return MAX_TELEOP_SPEED;
    }

    public static double getMaxRotationSpeed() {
        return MAX_ROTATION_SPEED;
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
        resetOdometry(pose);
    }

    // Updates pose based on encoder measurements
    // Runs periodically in Autonomous and Teleop
    public void updateOdometry() {
        // Get the rotation of the robot from the gyro.
        Rotation2d gyroAngle = ahrs.getRotation2d();

        // Update the pose
        pose = swerveDriveOdometry.update(
            gyroAngle,
            new SwerveModulePosition[] {
                frontLeft.getPosition(), frontRight.getPosition(),
                backLeft.getPosition(), backRight.getPosition()
            });
    }

    // Forces our pose to update to a new location
    // Ran at start of auto and whenever a valid AprilTag reading is received
    public void resetOdometry(Pose2d pose) {
        swerveDriveOdometry.resetPosition(getYaw(),
            new SwerveModulePosition[] { 
                frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
            },
            pose);
    }

    // Updates pose to AprilTag reading
    public void autoUpdateAprilTagPose(double x, double y, double rotatation, double error) {
        if (error > MAX_APRIL_TAG_ERROR) {
            return;
        }

        Pose2d aprilTagPose = new Pose2d(x, y, new Rotation2d(rotatation));
        resetOdometry(aprilTagPose);
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

    public double getX() {
        return pose.getX();
    }

    public double getY() {
        return pose.getY();
    }

    public double getZ() {
        return pose.getRotation().getRadians();
    }

    // Checks if all rotate motors are at setpoint
    private boolean allWheelsRotated() {
        return (frontLeft.rotateControllerAtSetpoint() && frontRight.rotateControllerAtSetpoint() &&
             backLeft.rotateControllerAtSetpoint() && backRight.rotateControllerAtSetpoint());
    }
   

    /***********************************************************************************************
     *
     *      TEST CODE
     *
     ***********************************************************************************************/
    public void testEncoders() {
        if (printCount % 50 == 0) {
            frontLeft.displayEncoderValues();
            //frontRight.displayEncoderValues();
            //backLeft.displayEncoderValues();
            //backRight.displayEncoderValues();
        }
        printCount++;
    }

    public void initTestWheelPower() {
        backLeft.initDriveMotorSlider();
        backLeft.initRotateMotorSlider();
    }
    
    public void testWheelPower() {
        backLeft.updateMotorPeriodic();
    }

    public void printPowerandVelocity() {
        //frontLeft.displayPowerAndVelocity();
        //frontRight.displayPowerAndVelocity();
        //backLeft.displayPowerAndVelocity();
        //backRight.displayPowerAndVelocity();
    }

    public void testPose() {
        if (printCount % 2 == 0) {
            System.out.println("X=" + getX() + " Y=" + getY() + " Z=" + getZ());
        }
        printCount++;
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