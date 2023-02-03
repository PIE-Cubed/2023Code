// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import frc.robot.*;
import frc.robot.commands.*;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * The start of the BasicAuto class
 */
public class BasicAuto extends SequentialCommandGroup {
    /**
     * The constructor for the BasicAuto class
     */
    public BasicAuto(Drive drive, PoseEstimation position, double delaySec) {
        // Starting position
        Pose2d startPos = new Pose2d(
            0.00,
            0.00,
            new Rotation2d(0.00)
        );

        // The ending position
        Pose2d endPose = new Pose2d(
            1.00,
            0.00,
            new Rotation2d(0.00)
        );

        // First set of movements
        List<Pose2d> move =
            List.of(
                startPos,
                endPose
            );

        //Adds commands to the group
        addCommands(
            new AutoDrive(drive, position, move)
        );
    }
}

// End of the BasicAuto class