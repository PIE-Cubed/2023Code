// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShootLocation;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Start of the CheckShooterReady class
 */
public class CheckShooterReady extends CommandBase {
    // Variables
    private boolean       isShooterReady = false;
    private ShootLocation location;

    // Object creation
    private Shooter shooter;

    /**
     * Constructor for the CheckShooterReady class
     *
     * @param subsystem The subsystem used by this command.
     */
    public CheckShooterReady(Shooter shooter, ShootLocation location) {
        // Localizes the parameters
        this.shooter  = shooter;
        this.location = location;

        // Declare subsystem dependencies
        addRequirements(this.shooter);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Checks if shooter ready
        isShooterReady = shooter.shooterReady();

        // Starts the shooter
        new StartShooter(shooter, location);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isShooterReady;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Prints that the command ran
        System.out.println("Shooter ready");
    }
}

// End of the CheckShooterReady class