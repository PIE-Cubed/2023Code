// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Start of the AutoEnd class
 */
public class AutoEnd extends CommandBase {
    // Object creation
    private Drive drive;
    private Shooter shooter;
    private Grabber grabber;
    private LedLights led;

    /**
     * Constructor for the AutoEnd class
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoEnd(Drive drive, Shooter shooter, Grabber grabber, LedLights led) {
        // Localizes the parameters
        this.led = led;

        // Declare subsystem dependencies
        addRequirements(this.led);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Led lights turn Gold
        led.autoModeFinished();

        // Stops applicable motors
        grabber.stopGrabber();
        grabber.grabberDeploy();
        shooter.disableShooter();
        drive.stopWheels();
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

// End of the AutoEnd class