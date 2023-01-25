package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;

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

    public Drive() {
        FRONT_LEFT_LOCATION  = new Translation2d(0, 0);
        FRONT_RIGHT_LOCATION = new Translation2d(0, 0);
        BACK_LEFT_LOCATION   = new Translation2d(0, 0);
        BACK_RIGHT_LOCATION  = new Translation2d(0, 0);

        frontLeft  = new SwerveModule(16, 17);
        frontRight = new SwerveModule(10, 11);
        backLeft   = new SwerveModule(14, 15);
        backRight  = new SwerveModule(12, 13);

        swerveDriveKinematics = new SwerveDriveKinematics(FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);
    }

}
