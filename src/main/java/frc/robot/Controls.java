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
	private final int    DRIVE_ID       = 0;

	// Values in meters, field-based pose Y coords
	private final double GRID_DIVIDER_1 = 1.905;
	private final double GRID_DIVIDER_2 = 3.581;
	private final double FIELD_WIDTH    = 8.0137;

	// Controller object declaration
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
					return new Pose2d(1.767, FIELD_WIDTH - 1.626, new Rotation2d(Math.PI));
				}
				else if (yLocation > (FIELD_WIDTH - GRID_DIVIDER_2)) {
					return new Pose2d(1.767, FIELD_WIDTH - 3.302, new Rotation2d(Math.PI));
				}
				else {
					return new Pose2d(1.767, FIELD_WIDTH - 4.978, new Rotation2d(Math.PI));
				}
			}
			else {
				if (yLocation < GRID_DIVIDER_1) {
					return new Pose2d(1.767, 1.626, new Rotation2d(Math.PI));
				}
				else if (yLocation < GRID_DIVIDER_2) {
					return new Pose2d(1.767, 3.302, new Rotation2d(Math.PI));
				}
				else {
					return new Pose2d(1.767, 4.978, new Rotation2d(Math.PI));
				}
			}
			
		}
		// Cube
		else if (driveController.getAButton()) {
			if (redSide) {
				if (yLocation > (FIELD_WIDTH - GRID_DIVIDER_1)) {
					return new Pose2d(1.767, FIELD_WIDTH - 1.067, new Rotation2d(Math.PI));
				}
				else if (yLocation > (FIELD_WIDTH - GRID_DIVIDER_2)) {
					return new Pose2d(1.767, FIELD_WIDTH - 2.743, new Rotation2d(Math.PI));
				}
				else {
					return new Pose2d(1.767, FIELD_WIDTH - 4.420, new Rotation2d(Math.PI));
				}
			}
			else {
				if (yLocation < GRID_DIVIDER_1) {
					return new Pose2d(1.767, 1.067, new Rotation2d(Math.PI));
				}
				else if (yLocation < GRID_DIVIDER_2) {
					return new Pose2d(1.767, 2.743, new Rotation2d(Math.PI));
				}
				else {
					return new Pose2d(1.767, 4.420, new Rotation2d(Math.PI));
				}
			}
			
		}
		// Right cone
		else if (driveController.getBButton()) {
			if (redSide) {
				if (yLocation > (FIELD_WIDTH - GRID_DIVIDER_1)) {
					return new Pose2d(1.767, FIELD_WIDTH - 0.508, new Rotation2d(Math.PI));
				}
				else if (yLocation > (FIELD_WIDTH - GRID_DIVIDER_2)) {
					return new Pose2d(1.767, FIELD_WIDTH - 2.184, new Rotation2d(Math.PI));
				}
				else {
					return new Pose2d(1.767, FIELD_WIDTH - 3.861, new Rotation2d(Math.PI));
				}
			}
			else {
				if (yLocation < GRID_DIVIDER_1) {
					return new Pose2d(1.767, 0.508, new Rotation2d(Math.PI));
				}
				else if (yLocation < GRID_DIVIDER_2) {
					return new Pose2d(1.767, 2.184, new Rotation2d(Math.PI));
				}
				else {
					return new Pose2d(1.767, 3.861, new Rotation2d(Math.PI));
				}
			}
		}
		else {
			return null;
		}
	}


	/****************************************************************************************** 
    *
    *    ARM FUNCTIONS
    * 
    ******************************************************************************************/



	/****************************************************************************************** 
    *
    *    LED FUNCTIONS
    * 
    ******************************************************************************************/
	/**
	 * Checks if the X button is pressed.
	 * 
	 * @return
	 */
	public boolean getCube() {
		return driveController.getLeftBumper();
	}

	/**
	 * Checks if the Y button is pressed.
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
		return xboxController.getStartButtonPressed();
	}
}
// End of the Controls class