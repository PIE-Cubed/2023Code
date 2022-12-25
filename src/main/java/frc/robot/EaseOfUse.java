// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Start of the EaseOfUse class
 */
public class EaseOfUse {
    /**
     * generate2dPose()
     * <p>Generates a Pose2d object using units more familiar to the team
     * <p>The double array should be {xPos, yPos, orientation}
     * 
     * @param coor Length Condenses the xPos (in feet)
     * @return Pose2D
     */
    public static Pose2d generate2dPose(double[] coor) {
        // Runs the generate2dPose method using a single array
        return generate2dPose(coor[0], coor[1], coor[2]);
    }

    /**
     * generate2dPose()
     * <p>Generates a Pose2d object using units more familiar to the team
     * 
     * @param xPos X position in feet
     * @param yPos Y position in feet
     * @param degrees Angle in degrees
     * @return Pose2D
     */
    public static Pose2d generate2dPose(double xPos, double yPos, double degrees) {
        // Converts the feet inputs into meter outputs
        double xMeters = ftToM(xPos);
        double yMeters = ftToM(yPos);

        // Generates a Rotation2d object based on degrees
        Rotation2d rot = generateRot2d(degrees);

        // Returns a Pose2d in the units required by the SwerveCommand
        return new Pose2d(xMeters, yMeters, rot);
    }

    /**
     * rotation2dDeg()
     * <p>Creates a Rotation2d instance using degrees passed to the method
     * 
     * @param degrees
     * @return Rotation2d
     */
    public static Rotation2d generateRot2d(double degrees) {
        double rad = Math.toRadians(degrees);
        return new Rotation2d(rad);
    }

    /**
     * ftToM()
     * <p>Converts feet to meters
     * 
     * @param feet
     * @return meters
     */
    private static double ftToM(double feet) {
        return Units.feetToMeters(feet);
    }
}
