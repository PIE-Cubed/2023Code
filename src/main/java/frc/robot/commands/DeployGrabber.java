// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Start of the DeployGrabber class
 */
public class DeployGrabber extends CommandBase {
    // Object creation
    private Grabber grabber;

    /**
     * Constructor for the DeployGrabber class
     *
     * @param subsystem The subsystem used by this command.
     */
    public DeployGrabber(Grabber grabber) {
        // Localizes the parameters
        this.grabber = grabber;

        // Declare subsystem dependencies
        addRequirements(this.grabber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Deploys the grabber
        grabber.grabberDeploy();
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
        System.out.println("Grabber deployed");
    }
}

// End of the DeployGrabber class