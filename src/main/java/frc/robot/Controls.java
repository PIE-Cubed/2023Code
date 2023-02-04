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
	private final int JOYSTICK_ID = 1;
	private final int XBOX_ID     = 0;

	// Controller object declaration
	private Joystick       joystick;
	private XboxController xboxController;

	// Rate limiters
	private SlewRateLimiter xLimiter;
	private SlewRateLimiter yLimiter;
	private SlewRateLimiter rotateLimiter;

	//Constructor
	public Controls() {
		// Instance Creation
		joystick       = new Joystick(JOYSTICK_ID);
		xboxController = new XboxController(XBOX_ID);

		// Create the rate limiters
		xLimiter      = new SlewRateLimiter(12); // -6 to 6 in a second
		yLimiter      = new SlewRateLimiter(12); // -6 to 6 in a second
		rotateLimiter = new SlewRateLimiter(2 * Math.PI); // -pi to pi in a second
	}

	/**
	 * DRIVE FUNCTIONS
	 */
	/**
	 * Gets the forward speed
	 * <p>Forward is positive to match chassis speed standards
	 * <p>This measures rotatation around the Y axis, which is effectively translation on the X axis
	 * 
	 * @return forwardSpeed
	 */
	public double getForwardSpeed() {
		double speed = -1 * joystick.getY() * Drive.getMaxSpeed();

		// If we are in deadzone, y is 0
		speed = MathUtil.applyDeadband(speed, 0.1, 1);

		// 
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
		double speed = -1 * joystick.getX() * Drive.getMaxSpeed();

		// If we are in deadzone, x is 0
		speed = MathUtil.applyDeadband(speed, 0.1, 1);

		//
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
		double speed = -1 * joystick.getZ() * Drive.getMaxRotationSpeed(); 

		// If we are in deadzone, rotatespeed is 0
		speed = MathUtil.applyDeadband(speed, 0.1, 1);
		
		//
		speed = rotateLimiter.calculate(speed);

		return speed;    
	}


	/**
	 * ARM CONTROLS
	 */



	/**
	 * WRIST CONTROLS
	 */



	/**
	 * CLAW CONTROLS
	 */



	/**
	 * Checks if the start button is pressed
	 * @return start button pressed
	 */
	public boolean autoKill() {
		return xboxController.getStartButtonPressed();
	}
}

// End of the Controls class