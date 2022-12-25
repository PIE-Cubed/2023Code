// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import frc.robot.commands.*;
import frc.robot.commands.CommandGroups.*;
import frc.robot.EaseOfUse;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.ShootLocation;
import frc.robot.subsystems.Grabber.GrabberDirection;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Start of the wall class
 */
public class Wall extends SequentialCommandGroup {
    /**
     * The constructor for that Wall class
     */
    public Wall(Drive drive, Shooter shooter, Grabber grabber, LedLights led, double delay) {
        // Starting position
        Pose2d startPos = EaseOfUse.generate2dPose(0.00, 0.00, 0.00);

        // Arrays with movement locations
        double[] end1  = {0.0, 3.0, 0.0};
        double[] end2  = {0.0, 1.25, 0.0};
        double[] end3  = {0.0, 2.5, 0.0};

        // First set of movements
        List<Pose2d> move1 =
            List.of(
                startPos,
                EaseOfUse.generate2dPose(end1)
            );

        // Second set of movements
        List<Pose2d> move2 =
            List.of(
                EaseOfUse.generate2dPose(end1),
                EaseOfUse.generate2dPose(end2)
            );

        // Third set of movements
        List<Pose2d> move3 =
            List.of(
                EaseOfUse.generate2dPose(end2),
                EaseOfUse.generate2dPose(end3)
            );

        //Adds commands to the group
        addCommands(
            new AutoInit(led, delay),
            new DeployGrabber(grabber),
            new GrabberControl(grabber, GrabberDirection.INTAKE),
            new AutoDrive(drive, move1),
            new DriveAndPrepShooter(drive, shooter, move2),
            new AutoShoot(drive, shooter, grabber, ShootLocation.AUTO_RING, 4),
            new RetractGrabber(grabber),
            new AutoDrive(drive, move3),
            new AutoEnd(drive, shooter, grabber, led)
        );
    }
}

// End of the Wall class