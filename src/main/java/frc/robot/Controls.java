// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.math.MathUtil;

/**
 * Start of the Controls class
 */
public class Controls {
	// CONSTANTS
	private final int XBOX_ID     = 0;
	private final int JOYSTICK_ID = 1;

	// Controller object declaration
	private Joystick       joystick;
	private XboxController xboxController;

	/**
	 * The constructor for the Controls class
	 */
	public Controls() {
		// Instance Creation
		joystick       = new Joystick(JOYSTICK_ID);
		xboxController = new XboxController(XBOX_ID);
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
		double speed = -1 * joystick.getY();

		// If we are in deadzone, y is 0
		speed = MathUtil.applyDeadband(speed, 0.1, 1);

		//
		speed *= Drive.getMaxDriveSpeed();

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
		double speed = -1 * joystick.getX();

		// If we are in deadzone, x is 0
		speed = MathUtil.applyDeadband(speed, 0.1, 1);

		//
		speed *= Drive.getMaxDriveSpeed();

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
		double speed = -1 * joystick.getZ(); 

		// If we are in deadzone, rotatespeed is 0
		speed = MathUtil.applyDeadband(speed, 0.15, 1);

		//
		speed *= Drive.getMaxRotateSpeed();

		//
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
    *    AUTOKILL
    * 
    ******************************************************************************************/
	/**
	 * Checks if the start button is pressed
	 * @return startButtonPressed
	 */
	public boolean autoKill() {
		return xboxController.getStartButtonPressed();
	}
}

// End of the Controls class