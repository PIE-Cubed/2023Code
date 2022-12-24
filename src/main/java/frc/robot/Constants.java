// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants.
 * This class should not be used for any other purpose.
 * All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever
 * the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Robot dimensions measured from the center of each wheel (inches)
    private static final double ROBOT_WIDTH_IN  = 0;
    private static final double ROBOT_LENGTH_IN = 0;

    // Robot dimensions measured from the center of each wheel (meters)
    private static final double ROBOT_WIDTH_M    = Units.inchesToMeters(ROBOT_WIDTH_IN);
    private static final double ROBOT_LENGTH_M   = Units.inchesToMeters(ROBOT_LENGTH_IN);

    // Translations based off the wheel distances from the center of the robot (meters)
    private static final Translation2d FL_TRANSLATION = new Translation2d(-ROBOT_WIDTH_M / 2, ROBOT_LENGTH_M / 2);
    private static final Translation2d RL_TRANSLATION = new Translation2d(-ROBOT_WIDTH_M / 2, -ROBOT_LENGTH_M / 2);
    private static final Translation2d FR_TRANSLATION = new Translation2d(ROBOT_WIDTH_M / 2, ROBOT_LENGTH_M / 2);
    private static final Translation2d RR_TRANSLATION = new Translation2d(ROBOT_WIDTH_M / 2, -ROBOT_LENGTH_M / 2);

    // Kinematics Creation
    public static final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(FL_TRANSLATION, RL_TRANSLATION, FR_TRANSLATION, RR_TRANSLATION);

    // Auto Speeds
    public static final double MAX_AUTO_SPEED                = 3; // Meters per second
    public static final double MAX_AUTO_ACCELERATION         = 3; // Meters per second per second
    public static final double MAX_AUTO_ANGULAR_SPEED        = Math.PI; // Radians per second
    public static final double MAX_AUTO_ANGULAR_ACCELERATION = Math.PI; // Radians per second per second

    // X Position PID
    public static final double xP = 1;
    public static final double xI = 0;
    public static final double xD = 0;

    // Y Position PID
    public static final double yP = 1;
    public static final double yI = 0;
    public static final double yD = 0;

    // Theta PID
    public static final double thetaP = 1;
    public static final double thetaI = 0;
    public static final double thetaD = 0;

    // Constraint for the motion profiled angle controller
    public static final TrapezoidProfile.Constraints thetaControllerConstraints =
        new TrapezoidProfile.Constraints(MAX_AUTO_ANGULAR_SPEED, MAX_AUTO_ANGULAR_ACCELERATION);

    // TeleOp Speeds
    public static final double MAX_TELEOP_SPEED = 3; // Meters per second

    // Module Speeds
    public static final double MAX_MODULE_ROTATE_SPEED        = 2 * Math.PI;
    public static final double MAX_MODULE_ROTATE_ACCELERATION = 2 * Math.PI;

    // Wheel Measurements
    public static final double wheelDiameterInches = 4.00;
    public static final double wheelDiameterMeters = Units.inchesToMeters(wheelDiameterInches);

    // Encoder Values
    public static final double ticksPerFoot        = 5.65;
    public static final double ticksPerRevolution  = ticksPerFoot / wheelDiameterInches / 12;

    // Distance Calculations
    public static final double inchesDrivenPerTick  = (wheelDiameterInches * Math.PI) / ticksPerRevolution;
    public static final double metersDrivenPerTick  = (wheelDiameterMeters * Math.PI) / ticksPerRevolution;
    public static final double degreesrotatePerTick = 360 / ticksPerRevolution;
    public static final double radiansrotatePerTick = (2 * Math.PI) / ticksPerRevolution;

    // Driving PID
    public static final double dP = 1;
    public static final double dI = 0;
    public static final double dD = 0;

    // Rotation PID
    public static final double rotateP = 1;
    public static final double rotateI = 0;
    public static final double rotateD = 0;
}
