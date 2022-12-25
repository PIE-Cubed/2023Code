// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.ShootLocation;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Start of the AutoShoot class
 */
public class AutoShoot extends SequentialCommandGroup {
    /**
     * Constructor for the AutoShoot class
     */
    public AutoShoot(Drive drive, Shooter shooter, Grabber grabber, ShootLocation location, int balls) {
        // Declare subsystem dependencies
        addRequirements(drive, shooter);

        // Adds commands to the group
        addCommands(
            new StartShooter(shooter, location),
            new CheckShooterReady(shooter, location),
            new Fire(shooter, grabber, location, balls),
            new RunCommand(
                () ->
                shooter.disableShooter(),
                shooter)
        );
    }
}

// End of the AutoShoot class