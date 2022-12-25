// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShootLocation;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Start of the StartShooter class
 */
public class StartShooter extends CommandBase {
    // Variables
    private ShootLocation location = ShootLocation.OFF;

    // Object creation
    private Shooter shooter;

    /**
     * Constructor for the StartShooter class
     *
     * @param subsystem The subsystem used by this command.
     */
    public StartShooter(Shooter shooter, ShootLocation location) {
        // Localizes the parameters
        this.shooter  = shooter;
        this.location = location;

        // Declare subsystem dependencies
        addRequirements(this.shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Starts the shooter
        shooter.shooterControl(location);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Prints the command completed
        System.out.println("Shooter started at " + location.toString() + "power" );
    }
}

// End of the StartShooter class