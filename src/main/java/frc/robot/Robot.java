// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.networktables.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Start of the Robot class
 */
public class Robot extends TimedRobot {
	// ERROR CODES
	public static final int FAIL = -1;
	public static final int PASS =  1;
	public static final int DONE =  2;
	public static final int CONT =  3;

	// Networktables
	private NetworkTable FMSInfo;
	private NetworkTableEntry isRedAlliance;

	// Object creation
	Controls controls;
	Drive    drive;

	// Variables
	private int status = CONT;

	// Auto path
	private static final String leftAuto = "Left";
	private static final String middleAuto   = "Middle";
	private static final String rightAuto = "Right";
	private String m_autoSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();

	// Auto Delay
	private int delaySec = 0;
	

	/**
	 * Constructor
	 */
	public Robot() {
		// Instance creation
		controls = new Controls();
		drive    = new Drive();

		//Creates a Network Tables instance
		FMSInfo = NetworkTableInstance.getDefault().getTable("FMSInfo");

		//Creates the Networktable Entry
		isRedAlliance = FMSInfo.getEntry("IsRedAlliance"); // Boolean
	}

	@Override
	/**
	 * robotInit()
	 * Runs once when the robot is started
	 */
	public void robotInit() {
		// Auto start location
		m_chooser.setDefaultOption("Left", leftAuto);
		m_chooser.addOption("Middle", middleAuto);
		m_chooser.addOption("Right", rightAuto);
		SmartDashboard.putData("Auto choices", m_chooser);

		// Auto delay
		SmartDashboard.putNumber("Auto delay seconds", 0);
	}

	@Override
	/**
	 * robotPeriodic()
	 * Runs every 20 miliseconds on the robot
	 */
	public void robotPeriodic() {
		// Nothing yet...
	}

	@Override
	/**
	 * autonomousInit()
	 * Runs once when Auto starts
	 */
	public void autonomousInit() {
		// Choses start position
		m_autoSelected = m_chooser.getSelected();
		System.out.println("Auto selected: " + m_autoSelected);

		// Gets the auto delay 
		delaySec = (int)SmartDashboard.getNumber("Auto delay seconds", 0);

		// Reset the gyro
	}

	@Override
	/**
	 * autonomousPeriodic()
	 * Runs every 20 miliseconds during Autonomous
	 */
	public void autonomousPeriodic() {
		long autoDelayMSec = delaySec * 1000;

		if (status == Robot.CONT) {
			switch (m_autoSelected) {
				default:
				status = DONE;
				break;
			}
    	}
	}

	@Override
	/**
	 * teleopInit()
	 * Runs once at the start of TeleOp
	 */
	public void teleopInit() {
		// Nothing yet...
	}

	@Override
	/**
	 * teleopPeriodic()
	 * Runs ever 20 miliseconds during TeleOp
	 */
	public void teleopPeriodic() {
		wheelControl();
	}

	@Override
	/**
	 * disabledInit()
	 * Runs once when the robot is disabled
	 */
	public void disabledInit() {
		// Nothing yet...   
	}

	@Override
	/**
	 * disabledPeriodic()
	 * Runs every 20 miliseconds while disabled.
	 * <p> This method should not do anything.
	 */
	public void disabledPeriodic() {
		// Nothing yet...
	}

	@Override
	/**
	 * testInit()
	 * Runs once at the start of Test
	 */
	public void testInit() {
		// Resets status
		status = Robot.CONT;
		drive.initTestWheelPower();
	}

	@Override
	/**
	 * testPeriodic()
	 * Runs constantly during test
	 */
	public void testPeriodic() {
		drive.testEncoders();
		drive.testWheelPower();
	}

	/**
	 * Controls the wheels in TeleOp
	 */
	private void wheelControl() {
		// Gets Joystick Values
		double forwardPower = controls.getForwardPower();
		double strafePower  = controls.getStrafePower();
		double rotatePower  = controls.getRotatePower();

		drive.teleopDrive(forwardPower, strafePower, rotatePower);
	}
}

// End of the Robot class