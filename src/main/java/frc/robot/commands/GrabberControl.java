// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Grabber.GrabberDirection;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Start of the GrabberControl class
 */
public class GrabberControl extends CommandBase {
    // Object creation
    private Grabber grabber;
    private GrabberDirection direction = GrabberDirection.OFF;

    /**
     * Constructor for the GrabberControl class
     *
     * @param subsystem The subsystem used by this command.
     */
    public GrabberControl(Grabber grabber, GrabberDirection direction) {
        // Localizes the parameters
        this.grabber   = grabber;
        this.direction = direction;

        // Declare subsystem dependencies
        addRequirements(this.grabber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Starts the grabber
        grabber.setGrabberMotor(direction);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Keeps the grabber running
        grabber.setGrabberMotor(direction);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Prints completion
        System.out.println("Grabber started. Direction " + direction.toString());
    }
}

// End of the GrabberControl class