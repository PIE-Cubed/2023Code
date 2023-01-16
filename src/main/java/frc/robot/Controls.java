// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

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

	//Constructor
	public Controls() {
		// Instance Creation
		joystick       = new Joystick(JOYSTICK_ID);
		xboxController = new XboxController(XBOX_ID);
	}

	/**
	 * DRIVE FUNCTIONS
	 */

	/**
	 * Positive values are from clockwise rotation and negative values are from counter-clockwise
	 * @return rotatePower
	 */
	public double getRotatePower() {
		double power = joystick.getZ(); 

		// If we are in deadzone, rotatepower is 0
		if ((Math.abs(power) < 0.3)) {
		power = 0;
		}

		// Cubes the power and clamps it because the rotate is SUPER sensitive
		power = Math.pow(power, 3.0);
		power = MathUtil.clamp(power, -.5, .5);

		return power;    
	}

	/**
	 * Gets the drive X
	 * @return driveX
	 */
	public double getDriveX() {
		double power = joystick.getX();

		// If we are in deadzone, x is 0
		if ((Math.abs(power) < 0.05)) {
		power = 0;
		}

		return power;
	}

	/**
	 * Gets the drive Y
	 * @return driveY
	 */
	public double getDriveY() {
		double power = joystick.getY() * -1;

		// If we are in deadzone, y is 0
		if ((Math.abs(power) < 0.05)) {
		power = 0;
		}

		return power;
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