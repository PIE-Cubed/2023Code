// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Start of the Controls class
 */
public class Controls {
	// CONSTANTS
	private final int CONTROLLER_ID = 0;

	private XboxController controller;

	/**
	 * The constructor for the Controls class
	 */
	public Controls() {
		// Instance Creation
		controller = new XboxController(CONTROLLER_ID);
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
	public double 
	getForwardSpeed() {
		double speed;
		double power = -1 * controller.getLeftY();
		power = Math.pow(power, 3);

		// Turns the power into a speed
		speed = power * Drive.MAX_DRIVE_SPEED;

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
		double power = -1 * controller.getLeftX();
		power = Math.pow(power, 3);

		// Turns the power into a speed
		speed = power * Drive.MAX_DRIVE_SPEED;

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
		double power = -1 * controller.getRightX();
		power = Math.pow(power, 3);

		// Turns the power into a speed
		speed = power * Drive.MAX_ROTATE_SPEED;

		return speed;
	}

	
	/****************************************************************************************** 
    *
    *    ARM FUNCTIONS
    * 
    ******************************************************************************************/
	// Left trigger intakes, right trigger outputs
	public double intakePower() {
		boolean output = controller.getLeftTriggerAxis() > 0;
		boolean intake = controller.getRightTriggerAxis() > 0;

		if (output) {
			return -0.2;
		}
		else if (intake) {
			return 0.08;
		}
		return 0;
	}

	public boolean startIntake() {
		return controller.getAButtonPressed();
	}
}
// End of the Controls class
