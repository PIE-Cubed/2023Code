// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Start of the LedLights class
 */
public class LedLights extends SubsystemBase {
	// Variables
	private boolean shooterOnTarget;
	private boolean limelightOnTarget;
	private boolean limelightNoTarget;
	private boolean errorCodeDisplayed;
	private int     delayCount;

	// CONSTANTS
	private final int LED_PWM_CHANNEL = 0;
	private final int LED_DELAY       = 50; 

	// Object creation
	private Spark ledController;

	/**
	 * The constructor for the LedLights class
	 */
	public LedLights()  {
		// Creates an instance for the Spark Controler
		ledController = new Spark(LED_PWM_CHANNEL);

		// Sets variables
		delayCount = 0;
		shooterOnTarget    = false;
		limelightOnTarget  = false;
		limelightNoTarget  = true;
		errorCodeDisplayed = false;

		// Sets the default color to team colors
		teamColors();
	}

	/**
	 * Team Colors
	 */
	public void teamColors() {
		// Sparkle blue on gold
		ledController.set(0.41);
		errorCodeDisplayed = false;
	}

	/**
	 * DEFAULT
	 */
	public void defaultMode( boolean isRedAlliance) {
		// Checkes if an error code has recently been displayed
		if (errorCodeDisplayed == true) {
			// Resets the delay counter and determines if an error code has been displayed
			errorCodeDisplayed = false;
			delayCount = 0;
		}
		else {
			// Increments the counter
			delayCount ++;
		}

		// If passed the delay amount (20 cycles of the code), reverts to alliance colors
		if (delayCount >= LED_DELAY) {
			if (isRedAlliance == true) {
				// Sets our colors to red
				redAlliance();
			}
			else if (isRedAlliance == false) {
				// Sets our colors to blue
				blueAlliance();
			}

			// Resets the counter
			delayCount = 0;
		}
	}

	public void redAlliance(){
		// Heartbeat Red (-0.25)
		ledController.set(-0.25);
		errorCodeDisplayed = false;
	}

	public void blueAlliance(){
		// Heartbeat Blue (-0.23)
		ledController.set(-0.23);
		errorCodeDisplayed = false;
	}

	/**
	 * AUTO MODES
	 */
	public void autoMode() {
		// Solid color set to Aqua
		ledController.set(0.81);
		errorCodeDisplayed = false;
	}

	public void autoModeFinished() {
		// Solid color set to Gold
		ledController.set(0.67);
		errorCodeDisplayed = true;
	}

	/**
	 * SHOOTER
	 */
	public void shooterReady() {
		shooterOnTarget = true;
		errorCodeDisplayed = true;
	}

	public void shooterNotReady() {
		shooterOnTarget = false;
		errorCodeDisplayed = true;
	}


	/**
	 * LIMELIGHT ERROR CODES
	 */
	public void limelightFinished() {
		limelightOnTarget = true;
		errorCodeDisplayed = true;
	}

	public void limelightAdjusting() {
		limelightOnTarget = false;
		errorCodeDisplayed = true;
	}

	public void limelightNoValidTarget() {
		limelightNoTarget = true;
		limelightOnTarget = false;
		errorCodeDisplayed = true;
	}
	
	// Green        - shooter up to speed and limelight targeted
	// Yellow       - shooter up to speed, but limelight not targeted
	// Red          - shooter not up to speed
	// Red flashing - limelight and shooter are both off	
	public void updateShooter() {
		if (shooterOnTarget && limelightOnTarget) {
			// Solid Green
			ledController.set(0.77);
		}
		else if (shooterOnTarget && limelightNoTarget) {
			// Solid Yellow
			ledController.set(0.69);
		}
		else if (shooterOnTarget) {
			// Solid Yellow
			ledController.set(0.69);
		}
		else if (!shooterOnTarget && limelightNoTarget) {
			// Strobe Red
			ledController.set(-0.11);
		}
		else if (!shooterOnTarget && limelightOnTarget) {
			// Solid Red
			ledController.set(0.61);
		}
		else if (!shooterOnTarget) {
			// Strobe Red
			ledController.set(-0.11);
		}
	}

	/**
	 * OBJECT TRACKING ERROR CODES
	 */
	public void cargoTrackingFinished() {
		// Solid Green
		ledController.set(0.77);
		errorCodeDisplayed = true;
	}

	public void cargoTrackingAdjusting() {
		// Solid Yellow
		ledController.set(0.69);
		errorCodeDisplayed = true;
	}

	public void cargoTrackingNoTarget() {
		// Solid Red
		ledController.set(0.61);
		errorCodeDisplayed = true;
	}

	/**
	 * CLIMBER ERROR CODES
	 */
	public void climberAtPosition() {
		// Solid Green
		ledController.set(0.77);
		errorCodeDisplayed = true;
	}

	public void climberMoving() {
		// Strobe gold
		ledController.set(-0.07);
		errorCodeDisplayed = true;
	}

	public void climberDone() {
		// Rainbow party palette
		ledController.set(-0.97);
		errorCodeDisplayed = false;

		// Sets delayCount to something absurd
		delayCount = (int)-1.0e100;
	}

	/**
	 * INIT FUNCTIONS
	 */
	public void autoInit() {
		delayCount = (int)-1.0e100;
	}

	public void teleopInit() {
		delayCount = 0;
	}
}

// End of the Ledligths Class