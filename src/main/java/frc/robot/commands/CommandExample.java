// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Start of the CommandExample class
 */
public class CommandExample extends CommandBase {
    // Object creation
    // Add a subsystem or two here

    /**
     * Constructor for the CommandExample class
     *
     * @param subsystem The subsystem used by this command.
     */
    public CommandExample(Subsystem subsystem) {
        // Localizes the parameters
        // Localize the subsystem

        // Declare subsystem dependencies
        // Add the subsystem as a requirement using addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Nothing yet...
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Nothing yet...
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Prints that the command ran
        System.out.println("");
    }
}

// End of the CommandExample class