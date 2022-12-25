// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.ShootLocation;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * Start of the DriveAndPrepShooter class
 */
public class DriveAndPrepShooter extends ParallelCommandGroup {
    /**
     * Constructor for the DriveAndPrepShooter class
     */
    public DriveAndPrepShooter(Drive drive, Shooter shooter, List<Pose2d> points) {
        // Declare subsystem dependencies
        addRequirements(drive, shooter);

        // Adds commands to the group
        addCommands(
            new AutoDrive(drive, points),
            new StartShooter(shooter, ShootLocation.HIGH_SHOT)
        );
    }
}

// End of the DriveAndPrepShooter class