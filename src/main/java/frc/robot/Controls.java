// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.subsystems.Shooter.ShootLocation;
import frc.robot.subsystems.Grabber.GrabberDirection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Start of the Controls class
 */
public class Controls {
    // CONSTANTS
    private final int XBOX_ID     = 0;
    private final int JOYSTICK_ID = 1;
    private final double LIMIT    = 0.50;

    // Controller object declaration
    private Joystick       joystick;
    private XboxController xboxController;

    // Rate limiters
    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter rotateLimiter;

    /**
     * The constructor for the Controls class
     */
    public Controls() {
        // Instance Creation
        joystick       = new Joystick(JOYSTICK_ID);
        xboxController = new XboxController(XBOX_ID);

        // Rate limiter
        xLimiter      = new SlewRateLimiter(LIMIT);
        yLimiter      = new SlewRateLimiter(LIMIT);
        rotateLimiter = new SlewRateLimiter(LIMIT);
    }

    /**
     * JOYSTICK FUNCTIONS
     */
    /**
     * Returns wheter to start the shooter
     * @return
     */
    public boolean getShooterEnable() {        
        return joystick.getTrigger();        
    }
    
    /**
     * DRIVE FUNCTIONS
     */
    /**
     * Gets the drive X
     * @return driveX
     */
    public double getDriveX() {
        // Gets the joystick X
        double power = joystick.getX();

        // Strafe lock removes deadzone and cubes power for more precision
        if (getStrafeLock() == true) {
            power = Math.pow(power, 3);
        }
        // If we are in deadzone or rotatelock is on, x is 0
        if ((Math.abs(power) < 0.05) || (getRotateLock() == true)) {
            power = 0;
        }

        // Prevents us from accelerating sideways too quickly
        power = xLimiter.calculate(power);
 
        return power;
    }

    /**
     * Gets the drive Y
     * @return driveY
     */
    public double getDriveY() {
        // Gets the joystick Y and negates the power because forward is negative
        double power = joystick.getY() * -1;

        // Strafe lock removes deadzone and cubes power for more precision
        if (getStrafeLock() == true) {
            power = Math.pow(power, 3);
        }
        // If we are in deadzone or rotatelock is on, y is 0
        else if ((Math.abs(power) < 0.05) || (getRotateLock() == true)) {
            power = 0;
        }

        // Prevents us from accelerating forward too quickly
        power = yLimiter.calculate(power);

        return power;
    }

    /**
     * Positive values are from clockwise rotation and negative values are from counter-clockwise
     * @return rotatePower
     */
    public double getRotatePower() {
        // Gets the joystick Z
        double power = joystick.getZ(); 

        // If we are in deadzone or strafelock is on, rotatepower is 0
        if ((Math.abs(power) < 0.3) || (getStrafeLock() == true)) {
            power = 0;
        }

        // Prevents us from accelerating in circles too quickly
        power = rotateLimiter.calculate(power);

        // Cubes the power and clamps it because the rotate is SUPER sensitive
        power = Math.pow(power, 3.0);
        power = MathUtil.clamp(power, -.5, .5);

        return power;    
    }

    /**
     * Checks if we are in strafe lock mode
     * @return joystick button 5
     */
    private boolean getStrafeLock() {
        return joystick.getRawButton(5);
    }

    /**
     * Checks if we are in rotate lock mode
     * @return joystick button 3
     */
    private boolean getRotateLock() {
        return joystick.getRawButton(3);
    }

    /**
     * Returns shoot location based off of trigger and button 4
     * @return shoot location
     */
    public ShootLocation getShootLocation() {
        if (joystick.getTrigger() && joystick.getRawButton(4)) {
            return ShootLocation.LOW_SHOT;
        }
        else if (joystick.getTrigger() && joystick.getRawButton(6)) {
            return ShootLocation.LAUNCH_PAD;
        }
        else if (joystick.getTrigger()) {
            return ShootLocation.HIGH_SHOT;
        }
        else {
            return ShootLocation.OFF;
        }
    }

    public boolean enableCargoTracking() {
        return joystick.getRawButton(8);
    }

    public boolean toggleFieldDrive() {
        return joystick.getRawButton(10);
    }

    /**
     * XBOX CONTROLLER FUNCTIONS
     */
    /**
     * Start Button Pressed
     * <p>KILLS ALL ACTIVE AUTO PROGRAMS!
     * @return startButtonPressed
     */
    public boolean autoKill() {
        return xboxController.getStartButtonPressed();
    }
    
    /**
     * Button A Pressed
     * @return buttonAPressed
     */
    public boolean grabberDeployRetract() {
        return xboxController.getAButtonPressed();
    }

    /**
     * Button B Pressed
     * @return
     */
    public boolean secureBalls() {
        return xboxController.getBButtonPressed();
    }

    /**
     * Button X Pressed
     * @return
     */
    public boolean releaseBalls() {
        return xboxController.getXButtonPressed();
    }

    /**
     * Button Y Pressed
     * @return buttonBPressed
     */
    public boolean startShooter() {
        return xboxController.getYButton();
    }
        
    /** 
     * Left Bumper Pressed
     * @return leftBumperPressed 
     */
    public boolean toggleBlueClaw() { 
        return xboxController.getLeftBumperPressed(); 
    }

    /**
     * Right Bumper Pressed 
     * @return rightBumperPressed 
     */
    public boolean toggleYellowClaw() { 
        return xboxController.getRightBumperPressed(); 
    }

    /**
     * Button 7 on the Joystick Pressed
     * @return buttonSevenPressed
     */
    public boolean getClimberMoveToBar2() {
        return joystick.getRawButton(7);
    }

    /**
     * Button 9 on the Joystick Pressed
     * @return buttonNinePressed
     */
    public boolean getClimberMoveToBar3() {
        return joystick.getRawButton(9);
    }

    /**
     * Button 11 on the Joystick Pressed
     * @return buttonElevenPressed
     */
    public boolean getClimberMoveToBar4() {
        return joystick.getRawButton(11);
    }

    /**
     * Gets the power for the climber
     * @return climberPower
     */
    public double getClimberPower() {
        final double POWER = 0.3;
        double leftY = -1 * xboxController.getLeftY();
        
        if (leftY >= 0.1) {
            return POWER;
        }
        else if (leftY <= -0.1) {
            return -1 * POWER;
        }
        else {
            return 0.00;
        }
    }

    /**
     * Resets the climber encoder
     * @return rightStickPressed
     */
    public boolean resetClimberEncoder() {
        return xboxController.getRightStickButtonPressed();
    }
     
    //Grabber Direction based off of D-Pad
    public GrabberDirection getGrabberDirection() {
        if (xboxController.getPOV() == 0) {
            return GrabberDirection.INTAKE;
        }
        else if (xboxController.getPOV() == 180) {
            return GrabberDirection.EXPEL;
        }
        else {
            return GrabberDirection.OFF;
        }
    }
}

// End of the Controls class