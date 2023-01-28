package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;

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

    // Constants
    private final Translation2d FRONT_LEFT_LOCATION;
    private final Translation2d FRONT_RIGHT_LOCATION;
    private final Translation2d BACK_LEFT_LOCATION;
    private final Translation2d BACK_RIGHT_LOCATION;
    private final double        MAX_TELEOP_SPEED = 1;

    public Drive() {
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
    }

    public void teleopDrive(double forward, double strafe, double rotationSpeed) {
        SwerveModuleState[] swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(forward, strafe, rotationSpeed));
        //SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_TELEOP_SPEED);
        frontLeft.move(swerveModuleStates[0]);
        frontRight.move(swerveModuleStates[1]);
        backLeft.move(swerveModuleStates[2]);
        backRight.move(swerveModuleStates[3]);
    }

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