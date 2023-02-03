// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Start of the Delay class
 */
public class Delay extends CommandBase {
    // Variables
    private double delaySec = 0;

    // Object creation
    private Timer timer;

    /**
     * Constructor for the Delay class
     *
     * @param delaySec
     */
    public Delay(double delaySec) {
        // Localizes the parameters
        this.delaySec = delaySec;

        // Instance creation
        timer = new Timer();
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
        return timer.hasElapsed(delaySec);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Prints that it completed
        System.out.println("Delay Completed");
    }
}

// End of the Delay class