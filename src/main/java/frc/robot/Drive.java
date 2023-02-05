package frc.robot;

import com.kauailabs.navx.frc.AHRS;

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
    private static final double MAX_TELEOP_SPEED     = 6; // Meters per second - velocity is generally 6x the power
    private static final double MAX_ROTATION_SPEED   = 2 * Math.PI; // Radians per second
    private static final double MAX_WHEEL_SPEED      = 6; // Meters per second
    private final double MAX_APRIL_TAG_ERROR         = 10;
    private final double AUTO_DRIVE_TOLERANCE        = 0.01;
    private final double AUTO_DRIVE_ROTATE_TOLERANCE = 0.075;

    // Instance Variables
    private int     rotateCount            = 0;
    private double  rotateTimeOut          = 0;
    private int     printCount             = 0;
    private boolean firstTimeAutoCrabDrive = true;
    private boolean firstTimeAutoRotate    = true;
    private double  encoderTarget;
    private int     autoPointIndex         = 0;
    private boolean autoPointFirstTime     = true;
    
    // NAVX
    public static AHRS ahrs;

    // Rate limiters for auto drive
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter rotateLimiter;
    
    // Auto drive to points X controller - need 2 controllers for X and Y for both setpoints
    private static final double adp = MAX_WHEEL_SPEED * 1 / 2; // 2 meters away --> full power
    private static final double adi = 0.2;
    private static final double add = 0;
    PIDController autoDriveXController;
    PIDController autoDriveYController;

    // Auto drive to points rotate controller
    private static final double adrp = MAX_ROTATION_SPEED * (1 / (2 * Math.PI)) ; // 2*Pi radians away --> full power
    private static final double adri = 0.2;
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
        xLimiter = new SlewRateLimiter(12);
        yLimiter = new SlewRateLimiter(12);
        rotateLimiter = new SlewRateLimiter(4 * Math.PI);

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
        swerveDriveOdometry   = new SwerveDriveOdometry(swerveDriveKinematics, ahrs.getRotation2d(), new SwerveModulePosition[]{ 
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
        });

        autoDriveXController = new PIDController(adp, adi, add);
        autoDriveXController.setTolerance(AUTO_DRIVE_TOLERANCE);

        autoDriveYController = new PIDController(adp, adi, add);
        autoDriveYController.setTolerance(AUTO_DRIVE_TOLERANCE);

        autoDriveRotateController = new PIDController(adrp, adri, adrd);
        autoDriveRotateController.setTolerance(AUTO_DRIVE_ROTATE_TOLERANCE);

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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_WHEEL_SPEED);
        frontLeft.move(swerveModuleStates[0]);
        frontRight.move(swerveModuleStates[1]);
        backLeft.move(swerveModuleStates[2]);
        backRight.move(swerveModuleStates[3]);
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

    // Autonomous function to drive to inputted list of points
    public int autoDriveToPoints(Pose2d[] listOfPoints) {
        // Function ends once we pass the last point
        if (autoPointIndex >= listOfPoints.length) {
            autoPointIndex = 0;
            autoPointFirstTime = true;
            return Robot.DONE;
        }

        // Calculating targetVelocity based on distance to targetPoint
        Pose2d targetPoint = listOfPoints[autoPointIndex];

        double targetXVelocity      = autoDriveXController.calculate(getX(), targetPoint.getX());
        double targetYVelocity      = autoDriveYController.calculate(getY(), targetPoint.getY());
        double targetRotateVelocity = autoDriveRotateController.calculate(getZ(), targetPoint.getRotation().getRadians());

        targetXVelocity = xLimiter.calculate(targetXVelocity);
        targetYVelocity = yLimiter.calculate(targetYVelocity);
        targetRotateVelocity = rotateLimiter.calculate(targetRotateVelocity);

        // This runs once for each point in the list
        if (autoPointFirstTime == true) {
            autoPointFirstTime = false;
            autoDriveXController.setSetpoint(targetPoint.getX());
            autoDriveYController.setSetpoint(targetPoint.getY());
            autoDriveRotateController.setSetpoint(targetPoint.getRotation().getRadians());

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
        }

        // Actual movement -  only if wheels are rotated
        if (allWheelsRotated()) {
            teleopDrive(targetXVelocity, targetYVelocity, targetRotateVelocity);
        }
        else {
            // Wheels will try to rotate to same angle, while drive motors cannot move
            teleopDrive(targetXVelocity/100, targetYVelocity/100, targetRotateVelocity/100);
        }

        // If X, Y, and Rotation are at target, moves on to next point
        if (autoDriveXController.atSetpoint() && autoDriveYController.atSetpoint() && autoDriveRotateController.atSetpoint()) {
            autoPointIndex++;
            autoPointFirstTime = true;
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

    public static double getMaxSpeed() {
        return MAX_TELEOP_SPEED;
    }

    public static double getMaxRotationSpeed() {
        return MAX_ROTATION_SPEED;
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
        swerveDriveOdometry.resetPosition(new Rotation2d(0),
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
        if (printCount % 50 == 0) {
            System.out.println("X=" + getX() + " Y=" + getY() + " Z=" + getZ());
        }
        printCount++;
    }

    public void testGyro() {
        if (printCount % 50 == 0) {
            System.out.println(ahrs.getRotation2d().getDegrees());
            //System.out.println("Yaw=" + String.format( "%.2f", ahrs.getYaw()) + " Roll=" + String.format( "%.2f", ahrs.getRoll()) + " Pitch=" + String.format( "%.2f", ahrs.getPitch()));
        }
        printCount++;
    }

    public void initTestDrivePower() {
        SmartDashboard.putNumber("Drive Power", 0);
    }

    public void periodicTestDrivePower() {
        double drivePower = SmartDashboard.getNumber("Drive Power", 0);
        testDrive(drivePower, 0, 0);

        if (printCount % 25 == 0) {
            System.out.println("Drive Speed: " + String.format(".3f", frontLeft.getDriveVelocity()));
        }
        printCount++;
    }
}

// End of the Drive class