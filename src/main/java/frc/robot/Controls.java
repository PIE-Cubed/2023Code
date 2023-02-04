// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
	 * Gets the forward speed
	 * <p>Forward is positive to match chassis speed standards
	 * <p>This measures rotatation around the Y axis, which is effectively translation on the X axis
	 * 
	 * @return forwardSpeed
	 */
	public double getForwardSpeed() {
		double speed = joystick.getY() * -1 * Drive.getMaxSpeed();

		// If we are in deadzone, y is 0
		if ((Math.abs(speed) < 0.3)) {
			speed = 0;
		}

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
		double speed = joystick.getX() * -1 * Drive.getMaxSpeed();

		// If we are in deadzone, x is 0
		if ((Math.abs(speed) < 0.3)) {
			speed = 0;
		}

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
		double speed = joystick.getZ() * -1 * Drive.getMaxRotationSpeed(); 

		// If we are in deadzone, rotatepower is 0
		if ((Math.abs(speed) < 0.5)) {
			speed = 0;
		}

		// Cubes the speed and clamps it because the rotate is SUPER sensitive
		speed = Math.pow(speed, 3.0);

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