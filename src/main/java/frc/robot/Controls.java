// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Start of the Controls class
 */
public class Controls {
	// CONSTANTS
	private final int DRIVE_ID = 1;
	private final int XBOX_ID  = 0;

	// Controller object declaration
	//private Joystick       joystick;
	private XboxController driveController;
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
		xboxController  = new XboxController(XBOX_ID);
		driveController = new XboxController(DRIVE_ID);

		// Create the rate limiters
		xLimiter      = new SlewRateLimiter(6); // -6 to 6 in two seconds
		yLimiter      = new SlewRateLimiter(6); // -6 to 6 in two seconds
		rotateLimiter = new SlewRateLimiter(6 * Math.PI);
	}

	/****************************************************************************************** 
    *
    *    DRIVE FUNCTIONS
    * 
    ******************************************************************************************/
	/**
	 * Gets the forward speed
	 * <p>Forward is positive to match chassis speed standards
	 * <p>This measures rotatation around the Y axis, which is effectively translation on the X axis
	 * 
	 * @return forwardSpeed
	 */
	public double getForwardSpeed() {
		double speed;
		double power = -1 * driveController.getLeftY();

		// If we are in deadzone, y is 0
		power = MathUtil.applyDeadband(power, 0.1, 1);

		// Turns the power into a speed
		speed = power * Drive.MAX_DRIVE_SPEED;

		// Limits the acceleration when under driver control
		speed = xLimiter.calculate(speed);

		return speed;
	}

	/**
	 * Gets the strafe speed
	 * <p>Left is positive to match chassis speed standards
	 * <p>This measures rotatation around the X axis, which is effectively translation on the Y axis
	 * 
	 * @return strafeSpeed
	 */
	public double getStrafeSpeed() {
		double speed;
		double power = -1 * driveController.getLeftX();

		// If we are in deadzone, x is 0
		power = MathUtil.applyDeadband(power, 0.1, 1);

		// Turns the power into a speed
		speed = power * Drive.MAX_DRIVE_SPEED;

		// Limits the acceleration when under driver control
		speed = yLimiter.calculate(speed);

		return speed;
	}

	/**
	 * Gets the rotate speed
	 * <p>Counterclockwise is positive to match chassis speed standards
	 * <p>This measures rotatation around the Z axis
	 * 
	 * @return rotateSpeed
	 */
	public double getRotateSpeed() {
		double speed;
		double power = -1 * driveController.getRightX();

		// If we are in deadzone, rotatespeed is 0
		power = MathUtil.applyDeadband(power, 0.15, 1);

		// Turns the power into a speed
		speed = power * Drive.MAX_ROTATE_SPEED;

		// Limits the acceleration when under driver control
		speed = rotateLimiter.calculate(speed);

		return speed;    
	}


	/****************************************************************************************** 
    *
    *    ARM FUNCTIONS
    * 
    ******************************************************************************************/



	/****************************************************************************************** 
    *
    *    WRIST FUNCTIONS
    * 
    ******************************************************************************************/



	/****************************************************************************************** 
    *
    *    CLAW FUNCTIONS
    * 
    ******************************************************************************************/



	/****************************************************************************************** 
    *
    *    LED FUNCTIONS
    * 
    ******************************************************************************************/
	public boolean getCone() {
		return driveController.getYButton();
	}

	public boolean getCube() {
		return driveController.getXButton();
	}

	/**
	 * Checks if the start button is pressed
	 * @return startButtonPressed
	 */
	public boolean autoKill() {
		return xboxController.getStartButtonPressed();
	}
}

// End of the Controls class