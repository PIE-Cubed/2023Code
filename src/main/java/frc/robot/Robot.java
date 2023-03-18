// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * Start of the Robot class
 */
public class Robot extends TimedRobot {
	// ERROR CODES
	public static final int FAIL = -1;
	public static final int PASS =  1;
	public static final int DONE =  2;
	public static final int CONT =  3;

	// Object creation
	Controls       controls;
	Drive          drive;
	Claw           claw;

	/**
	 * Constructor
	 */
	public Robot() {
		// Instance creation
		drive    = new Drive();
		controls = new Controls();
		claw     = new Claw();
	}

	@Override
	/**
	 * robotInit()
	 * Runs once when the robot is started.
	 */
	public void robotInit() {
	}

	@Override
	/**
	 * robotPeriodic()
	 * Runs every 20 miliseconds on the robot.
	 */
	public void robotPeriodic() {
	}

	@Override
	/**
	 * autonomousInit()
	 * Runs once when the robot enters Autonomous mode.
	 */
	public void autonomousInit() {
	}

	@Override
	/**
	 * autonomousPeriodic()
	 * Runs every 20 miliseconds during Autonomous.
	 */
	public void autonomousPeriodic() {
	}

	@Override
	/**
	 * teleopInit()
	 * Runs once at the start of TeleOp.
	 */
	public void teleopInit() {
		// Nothing yet...
	}

	@Override
	/**
	 * teleopPeriodic()
	 * Runs ever 20 miliseconds during TeleOp.
	 */
	public void teleopPeriodic() {
		wheelControl();
		clawControl();
	}

	@Override
	/**
	 * disabledInit()
	 * Runs once when the robot enteres Disabled mode.
	 */
	public void disabledInit() {
		// Nothing yet...
	}

	@Override
	/**
	 * disabledPeriodic()
	 * Runs every 20 miliseconds while disabled.
	 * This method should not do anything.
	 */
	public void disabledPeriodic() {
		// Nothing yet...
	}

	@Override
	/**
	 * testInit()
	 * Runs once when the robot enters Test mode.
	 */
	public void testInit() {
	}

	@Override
	/**
	 * testPeriodic()
	 * Runs every 20 miliseconds during Test.
	 */
	public void testPeriodic() {
	}

	/**
	 * Controls the wheels in TeleOp
	 */
	private void wheelControl() {
		// Gets Joystick Values
		double  forwardSpeed      = controls.getForwardSpeed();
		double  strafeSpeed       = controls.getStrafeSpeed();
		double  rotateSpeed       = controls.getRotateSpeed();
		
		drive.teleopDrive(forwardSpeed, strafeSpeed, rotateSpeed, true);
	}

	public void clawControl() {
		boolean startIntake = controls.
		double power = controls.intakePower();
		claw.setIntake(power);
		if (Math.abs(power) > 0) {
			System.out.println(claw.resistance());
		}
	}
}
// End of the Robot class
