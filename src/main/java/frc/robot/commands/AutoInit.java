// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Start of the AutoInit class
 */
public class AutoInit extends CommandBase {
    // Variables
    private double delaySec = 0;

    // Object creation
    private Timer timer;
    private LedLights led;

    /**
     * Constructor for the AutoInit class
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoInit(LedLights led, double delay) {
        // Localizes the parameters
        this.led      = led;
        this.delaySec = delay;

        // Instance creation
        timer = new Timer();

        // Declare subsystem dependencies
        addRequirements(this.led);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Clamps the wait timer
        MathUtil.clamp(delaySec, 0.00, 5.00);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Resets the NavX Yaw
        Drive.ahrs.zeroYaw();
        
        // Sets the LedLights to Aqua
        led.autoMode();
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
        System.out.println("AutoInit Completed");
    }
}

// End of the AutoInit class