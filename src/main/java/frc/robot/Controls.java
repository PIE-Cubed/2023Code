// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Start of the Controls class
 */
public class Controls {
	// CONSTANTS
	private final int DRIVE_ID = 0;
	private final int ARM_ID   = 1;

	// Values in meters, field-based pose Y coords
	private final double GRID_DIVIDER_1 = 1.905;
	private final double GRID_DIVIDER_2 = 3.581;
	private final double FIELD_WIDTH    = 8.0137;

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
		SHELF,
		REST,
		CHUTE,
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
	 * Gets target location for placing objects in the grid
	 * When in front of 1 of the 3 sets, holding X, A, and B will
	 * align with the left cones, cubes, or right cones
	 * Returns null if nothing is held
	 * @return location
	 */
	public Pose2d getPlacementLocation(double yLocation, boolean redSide) {
		// Left cone
		if (driveController.getXButton()) {
			if (redSide) {
				if (yLocation > (FIELD_WIDTH - GRID_DIVIDER_1)) {
					return new Pose2d(1.767+0.150, FIELD_WIDTH - 1.626, new Rotation2d(Math.PI));
				}
				else if (yLocation > (FIELD_WIDTH - GRID_DIVIDER_2)) {
					return new Pose2d(1.767+0.150, FIELD_WIDTH - 3.302, new Rotation2d(Math.PI));
				}
				else {
					return new Pose2d(1.767+0.150, FIELD_WIDTH - 4.978, new Rotation2d(Math.PI));
				}
			}
			else {
				if (yLocation < GRID_DIVIDER_1) {
					return new Pose2d(1.767+0.150, 1.626, new Rotation2d(Math.PI));
				}
				else if (yLocation < GRID_DIVIDER_2) {
					return new Pose2d(1.767+0.150, 3.302, new Rotation2d(Math.PI));
				}
				else {
					return new Pose2d(1.767+0.150, 4.978, new Rotation2d(Math.PI));
				}
			}
			
		}
		// Cube
		else if (driveController.getAButton()) {
			if (redSide) {
				if (yLocation > (FIELD_WIDTH - GRID_DIVIDER_1)) {
					return new Pose2d(1.767+0.150, FIELD_WIDTH - 1.067, new Rotation2d(Math.PI));
				}
				else if (yLocation > (FIELD_WIDTH - GRID_DIVIDER_2)) {
					return new Pose2d(1.767+0.150, FIELD_WIDTH - 2.743, new Rotation2d(Math.PI));
				}
				else {
					return new Pose2d(1.767+0.150, FIELD_WIDTH - 4.420, new Rotation2d(Math.PI));
				}
			}
			else {
				if (yLocation < GRID_DIVIDER_1) {
					return new Pose2d(1.767+0.150, 1.067, new Rotation2d(Math.PI));
				}
				else if (yLocation < GRID_DIVIDER_2) {
					return new Pose2d(1.767+0.150, 2.743, new Rotation2d(Math.PI));
				}
				else {
					return new Pose2d(1.767+0.150, 4.420, new Rotation2d(Math.PI));
				}
			}
			
		}
		// Right cone
		else if (driveController.getBButton()) {
			if (redSide) {
				if (yLocation > (FIELD_WIDTH - GRID_DIVIDER_1)) {
					return new Pose2d(1.767+0.150, FIELD_WIDTH - 0.508, new Rotation2d(Math.PI));
				}
				else if (yLocation > (FIELD_WIDTH - GRID_DIVIDER_2)) {
					return new Pose2d(1.767+0.150, FIELD_WIDTH - 2.184, new Rotation2d(Math.PI));
				}
				else {
					return new Pose2d(1.767+0.150, FIELD_WIDTH - 3.861, new Rotation2d(Math.PI));
				}
			}
			else {
				if (yLocation < GRID_DIVIDER_1) {
					return new Pose2d(1.767+0.150, 0.508, new Rotation2d(Math.PI));
				}
				else if (yLocation < GRID_DIVIDER_2) {
					return new Pose2d(1.767+0.150, 2.184, new Rotation2d(Math.PI));
				}
				else {
					return new Pose2d(1.767+0.150, 3.861, new Rotation2d(Math.PI));
				}
			}
		}
		else {
			return null;
		}
	}

	/*
	 * Any press on the d-pad will lock the wheels for balancing
	 */
	public boolean lockWheels() {
		return (driveController.getPOV() != -1);
	}
	
	/*
	 * Pressing left joystick will zero yaw in case of emergency
	 */
	public boolean zeroYaw() {
		return driveController.getLeftStickButtonPressed();
	}
	
	/*
	 * Holding right trigger will enable precision control
	 */
	public boolean enablePrecisionDrive() {
		return driveController.getRightTriggerAxis() > 0.05;
	}

	
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
			if (armController.getRightBumperPressed()) {
				currentObject = Objects.CONE;
			}
			else if (armController.getLeftBumperPressed()) {
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
			armState = (getClawState() == Objects.CONE)? ArmStates.MID_CONE : ArmStates.MID_CUBE;
		}
		else if (armController.getYButton()) {
			armState = (getClawState() == Objects.CONE)? ArmStates.TOP_CONE : ArmStates.TOP_CUBE;
		}
		else if (armController.getPOV() == 270) {
			armState = ArmStates.SHELF;
		}
		else if (armController.getPOV() == 90) {
			armState = ArmStates.CHUTE;
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
