// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Start of the Controls class
 */
public class Controls {
	// CONSTANTS
	private final int DRIVE_ID = 0;
	private final int ARM_ID   = 1;

	// Controller object declaration
	private XboxController driveController;
	private XboxController armController;

	// Rate limiters
	private SlewRateLimiter xLimiter;
	private SlewRateLimiter yLimiter;
	private SlewRateLimiter rotateLimiter;

	// Enumeration for which object the claw is holding
	public enum Objects {
		CONE,
		CUBE,
		EMPTY
	};
	public static Objects currentObject;

	// Enumeration for which position the arm is at
	public enum ArmStates {
		TOP_CONE,
		TOP_CUBE,
		MID_CONE,
		MID_CUBE,
		BOT_CONE,
		BOT_CUBE,
		REST,
		GRAB
	};
	public static ArmStates armState;

	/**
	 * The constructor for the Controls class
	 */
	public Controls() {
		// Instance Creation
		driveController = new XboxController(DRIVE_ID);
		armController   = new XboxController(ARM_ID);

		// Create the rate limiters
		xLimiter      = new SlewRateLimiter(6); // -6 to 6 in two seconds
		yLimiter      = new SlewRateLimiter(6); // -6 to 6 in two seconds
		rotateLimiter = new SlewRateLimiter(6 * Math.PI);

		currentObject = Objects.EMPTY;
		armState      = ArmStates.REST;
	}

	/****************************************************************************************** 
    *
    *    DRIVE FUNCTIONS
    * 
    ******************************************************************************************/


	
	/****************************************************************************************** 
    *
    *    ARM FUNCTIONS
    * 
    ******************************************************************************************/
	/**
	 * Finds which object the claw should be holding.
	 * If the state is not empty, the arm class should close the claw.
	 * Cone and cube have different weight, so the arm should know which one we are holding.
	 * @return currentObject
	 */
	public Objects getClawState() {
		// If claw is empty, pressing a bumper will grab an object
		if (currentObject == Objects.EMPTY) {
			if (armController.getLeftBumperPressed()) {
				currentObject = Objects.CONE;
			}
			else if (armController.getRightBumperPressed()) {
				currentObject = Objects.CUBE;
			}
		}
		// If claw is not empty, pressing a bumper will release the object
		else {
			if (armController.getLeftBumperPressed() || armController.getRightBumperPressed()) {
				currentObject = Objects.EMPTY;
			}
		}

		return currentObject;
	}

	/**
	 * D-pad controls manual movement of wrist
	 * Up on D-pad is positive power (toward front of robot), down on D-pad is negative power
	 * @return manualPower
	 */
	public double getManualWristPower() {
		// Higher power if we are grabbing heavier object
		double manualPower;
		if (getClawState() == Objects.EMPTY) {
			manualPower = 0.06;
		}
		else if (getClawState() == Objects.CONE) {
			manualPower = 0.18;
		}
		else {
			manualPower = 0.12;
		}

		// Up on D-pad
		if (armController.getPOV() == 0) {
			return manualPower;
		}
		else if (armController.getPOV() == 180) {
			return -1 * manualPower;
		}
		else {
			return 0;
		}
	}

	public ArmStates getArmState() {
		if (armController.getAButton()) {
			armState = (getClawState() == Objects.CONE)? ArmStates.BOT_CONE : ArmStates.BOT_CUBE;
		}
		else if (armController.getBButton()) {
			armState = (getClawState() == Objects.CONE)? ArmStates.MID_CONE : ArmStates.MID_CUBE;
		}
		else if (armController.getYButton()) {
			armState = (getClawState() == Objects.CONE)? ArmStates.TOP_CONE : ArmStates.TOP_CUBE;
		}
		else if (armController.getPOV() == 90) {
			armState = ArmStates.REST;
		}
		else if (armController.getPOV() == 270) {
			armState = ArmStates.GRAB;
		}
		return armState;
	}


	/****************************************************************************************** 
    *
    *    LED FUNCTIONS
    * 
    ******************************************************************************************/
	

	
	/****************************************************************************************** 
    *
    *    MISC FUNCTIONS
    * 
    ******************************************************************************************/
	/**
	 * Checks if the left stick is pressed.
	 * 
	 * @return
	 */
	public boolean zeroYaw() {
		return driveController.getLeftStickButtonPressed();
	}

	/**
	 * Checks if the start button is pressed
	 * @return start button pressed
	 */
	public boolean autoKill() {
		return driveController.getStartButtonPressed();
	}
}

// End of the Controls class