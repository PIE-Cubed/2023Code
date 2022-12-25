// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.ShootLocation;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Start of the Fire class
 */
public class Fire extends CommandBase {
    // Variables
    private int balls = 0;
    private ShootLocation location = ShootLocation.OFF;
 
    // Object creation
    private Timer timer;
    private Shooter shooter;
    private Grabber grabber;

    /**
     * Constructor for the Fire class
     *
     * @param subsystem The subsystem used by this command.
     */
    public Fire(Shooter shooter, Grabber grabber, ShootLocation location, int balls) {
        // Localizes the parameters
        this.balls    = balls;
        this.shooter  = shooter;
        this.grabber  = grabber;
        this.location = location;

        // Instance creation
        timer = new Timer();

        // Declare subsystem dependencies
        addRequirements(this.shooter, this.grabber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Retracts the shooter pistons to fire balls
        shooter.openShooter();
        grabber.releaseBalls();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Keeps the shooter running
        new StartShooter(shooter, location);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(balls);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Prints that the command ran
        System.out.println(balls + "balls fired");
    }
}

// End of the Fire class