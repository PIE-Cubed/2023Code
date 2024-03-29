// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot.GrabberStates;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;

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

	private boolean lastTriggerPressed = false;

	// Limit button on claw
	private DigitalInput limitButton;

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
		SHELF,
		REST,
		CHUTE_CONE,
		CHUTE_CUBE,
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

		limitButton = new DigitalInput(0);

		// Create the rate limiters
		xLimiter      = new SlewRateLimiter(12); // -6 to 6 in two seconds
		yLimiter      = new SlewRateLimiter(12); // -6 to 6 in two seconds
		rotateLimiter = new SlewRateLimiter(6 * Math.PI);

		currentObject   = Objects.EMPTY;
		armState        = ArmStates.REST;
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
		power = Math.pow(power, 3);

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
		power = Math.pow(power, 3);

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
		power = Math.pow(power, 3);

		// Turns the power into a speed
		speed = power * Drive.MAX_ROTATE_SPEED;

		// Limits the acceleration when under driver control
		speed = rotateLimiter.calculate(speed);

		return speed;
	}

	/**
	 * Pressing and holding the Y button will automatically align with a gamepiece.
	 * 
	 * @return YButton
	 */
	public boolean allignWithPiece() {
		return driveController.getYButton();
	}

	/**
	 * Any press on the d-pad will lock the wheels for balancing
	 * 
	 * @return lockWheels
	 */
	public boolean lockWheels() {
		return (driveController.getPOV() == 90 || driveController.getPOV() == 270);
	}

	public double robotOrientedSpeed() {
		if (driveController.getPOV() == 0) {
			return 0.7;
		}
		else if (driveController.getPOV() == 180) {
			return -1;
		}
		return 0;
	}
	
	/**
	 * Pressing left joystick will zero yaw in case of emergency
	 * 
	 * @return zeroYaw
	 */
	public boolean zeroYaw() {
		return driveController.getLeftStickButtonPressed();
	}
	
	/**
	 * Holding right trigger will enable precision control
	 * 
	 * @return precisionControl
	 */
	public boolean enablePrecisionDrive() {
		return driveController.getRightTriggerAxis() > 0.05;
	}

	
	/****************************************************************************************** 
    *
    *    ARM FUNCTIONS
    * 
    ******************************************************************************************/
	public boolean getLeftBumper() {
		return armController.getLeftBumper() || allignWithPiece();
	}

	public boolean getRightBumper() {
		return armController.getRightBumper();
	}

	public boolean getLeftBumperPressed() {
		return armController.getLeftBumperPressed();
	}

	public boolean getRightBumperPressed() {
		return armController.getRightBumperPressed();
	}

	public boolean getEject() {
		return (armController.getRightTriggerAxis() > 0.01);
	}

	public boolean getLaunch() {
		return (armController.getLeftTriggerAxis() > 0.01);
	}

	/**
	 * Returns the inputted arm state based on controller input and the current object we are holding
	 * This state is only accepted under certain conditions (we must pass through rest between any 2 positions)
	 * @return armState
	 */
	public ArmStates getArmState() {
		if (armController.getAButton()) {
			armState = ArmStates.GRAB;
		}
		else if (armController.getXButton()) {
			armState = ArmStates.REST;
		}
		else if (armController.getBButton()) {
			armState = (Robot.grabberState == Robot.GrabberStates.HOLDING_CONE)? ArmStates.MID_CONE : ArmStates.MID_CUBE;
		}
		else if (armController.getYButton()) {
			armState = (Robot.grabberState == Robot.GrabberStates.HOLDING_CONE)? ArmStates.TOP_CONE : ArmStates.TOP_CUBE;
		}
		else if (armController.getPOV() == 270) {
			armState = ArmStates.SHELF;
		}
		else if (armController.getStartButton()) {
			armState = ArmStates.CHUTE_CONE;
		}
		else if (armController.getBackButton()) {
			armState = ArmStates.CHUTE_CUBE;
		}
		return armState;
	}

	/**
	 * D-pad controls manual movement of wrist
	 * Up on D-pad is positive power (toward front of robot), down on D-pad is negative power
	 * @return manualPower
	 */
	public double getManualWristPower() {
		// Higher power if we are grabbing heavier object
		double manualPower;
		if (Robot.grabberState == GrabberStates.HOLDING_CONE) {
			manualPower = 0.32;
		}
		else if (Robot.grabberState == GrabberStates.HOLDING_CUBE) {
			manualPower = 0.20;
		}
		else {
			manualPower = 0.12;
		}

		// Up on D-pad
		if (armController.getPOV() == 0) {
			return -1 * manualPower;
		}
		else if (armController.getPOV() == 180) {
			return manualPower;
		}
		else {
			return 0;
		}
	}

	public Objects getClawState() {
		return currentObject;
	}

	/**
	 * Checks if the limit switch is activated.
	 * @return limitSwitch
	 */
	public boolean getLimitSwitch() {
		return !limitButton.get();
	}

	/*
	 * These two functions implement logic of only activating once upon pressing or releasing trigger
	 */
	public boolean getArmTriggerPressed() {
		boolean triggerPressed = (armController.getLeftTriggerAxis() > 0.01 || armController.getRightTriggerAxis() > 0.01);

		if (triggerPressed && (!lastTriggerPressed)) {
			lastTriggerPressed = triggerPressed;
			return true;
		}
		else {
			lastTriggerPressed = triggerPressed;
			return false;
		}
	}

	public boolean getArmTriggerReleased() {
		boolean triggerOff = (armController.getLeftTriggerAxis() <= 0.01 && armController.getRightTriggerAxis() <= 0.01);

		if (triggerOff && lastTriggerPressed) {
			lastTriggerPressed = !triggerOff;
			return true;
		}
		else {
			lastTriggerPressed = !triggerOff;
			return false;
		}
	}


	/****************************************************************************************** 
    *
    *    LED FUNCTIONS
    * 
    ******************************************************************************************/
	/**
	 * Checks if the L bumper is pressed.
	 * 
	 * @return
	 */
	public boolean getCube() {
		return driveController.getLeftBumper();
	}

	/**
	 * Checks if the R bumper is pressed.
	 * 
	 * @return
	 */
	public boolean getCone() {
		return driveController.getRightBumper();
	}


	/****************************************************************************************** 
    *
    *    MISC FUNCTIONS
    * 
    ******************************************************************************************/
	/**
	 * Checks if the start button is pressed.
	 * 
	 * @return startButtonPressed
	 */
	public boolean autoKill() {
		return driveController.getStartButtonPressed();
	}
}
// End of the Controls class
