// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import frc.robot.commands.*;
import frc.robot.commands.CommandGroups.*;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.ShootLocation;
import frc.robot.subsystems.Grabber.GrabberDirection;

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
        double[] start = {0.0, 0.0, 0.0};

        // First set of movements
        double[] end1  = {0.0, 3.0, 0.0};

        // Second set of movements
        double[] end2   = {0.0, 1.25, 0.0};

        // Third set of movements
        double[] end3   = {0.0, 2.5, 0.0};

        //Adds commands to the group
        addCommands(
            new AutoInit(led, delay),
            new DeployGrabber(grabber),
            new GrabberControl(grabber, GrabberDirection.INTAKE),
            new DriveDistance(drive, start, end1),
            new DriveAndPrepShooter(drive, shooter, end1, end2),
            new AutoShoot(drive, shooter, grabber, ShootLocation.AUTO_RING, 4),
            new RetractGrabber(grabber),
            new DriveDistance(drive, end2, end3),
            new AutoEnd(drive, shooter, grabber, led)
        );
    }
}

// End of the Wall class