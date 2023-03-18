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
	private long stopOutput = 0;

	public enum ClawState {
		INTAKE,
		OUTPUT,
		STOP
	};
	public static ClawState clawState = ClawState.STOP;

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
	public ClawState getClawState() {
		if (controller.getAButtonPressed()) {
			// If A button is pressed, switch between intake and stop
			if (clawState == ClawState.INTAKE) {
				clawState = ClawState.STOP;
			}
			else {
				clawState = ClawState.INTAKE;
			}
		}
		else if (controller.getBButtonPressed()) {
			// If B button is pressed, switch between output and stop
			if (clawState == ClawState.OUTPUT) {
				clawState = ClawState.STOP;
			}
			else {
				clawState = ClawState.OUTPUT;
			}
		}
		return clawState;
	}

	public void updateButton(boolean buttonPressed) {
		if (buttonPressed && clawState == ClawState.INTAKE) {
			clawState = ClawState.STOP;
		}
	}
}
// End of the Controls class
