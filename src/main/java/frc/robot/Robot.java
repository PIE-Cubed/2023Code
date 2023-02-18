// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import java.nio.file.Path;
import java.nio.file.Paths;

import edu.wpi.first.math.geometry.Pose2d;
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
	Auto     auto;
	LED      led;

	// Variables
	private int status = CONT;
	private int count = 0;

	private long coneFlashEnd = 0;
	private long cubeFlashEnd = 0;

	// Auto path
	private static final String wallAuto   = "Wall";
	private static final String rampAuto   = "Ramp";
	private static final String centerAuto = "Center";
	private String m_autoSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();

	// Auto number of objects
	private int m_objectsPlaced;
	private final SendableChooser<Integer> m_objectChooser = new SendableChooser<>();

	// Auto Delay
	private long delaySec = 0;

	/**
	 * Constructor
	 */
	public Robot() {
		// Instance creation
		controls = new Controls(true);
		drive    = new Drive();
		auto     = new Auto(drive);
		led      = LED.getInstance();

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
		m_chooser.setDefaultOption("Wall", wallAuto);
		m_chooser.addOption("Ramp", rampAuto);
		m_chooser.addOption("Center", centerAuto);
		SmartDashboard.putData("Auto choices", m_chooser);

		// Auto objects placed
		m_objectChooser.setDefaultOption("1", 1);
		m_objectChooser.addOption("2", 2);
		m_objectChooser.addOption("3", 3);
		SmartDashboard.putData("Auto objects placed", m_objectChooser);

		// Auto delay
		SmartDashboard.putNumber("Auto delay seconds", 0);
	}

	@Override
	/**
	 * robotPeriodic()
	 * Runs every 20 miliseconds on the robot
	 */
	public void robotPeriodic() {
	}

	@Override
	/**
	 * autonomousInit()
	 * Runs once when Auto starts
	 */
	public void autonomousInit() {
		// Choses start position
		m_autoSelected = m_chooser.getSelected();

		// Gets the auto delay 
		delaySec = (long)SmartDashboard.getNumber("Auto delay seconds", 0);

		// Gets the number of objects placed in auto
		m_objectsPlaced = m_objectChooser.getSelected();

		//
		drive.resetYaw();
	}

	@Override
	/**
	 * autonomousPeriodic()
	 * Runs every 20 miliseconds during Autonomous
	 */
	public void autonomousPeriodic() {
		if (status == Robot.CONT) {
			switch (m_autoSelected) {
				case "Wall":
					status = auto.wallAuto(isRedAlliance.getBoolean(true), 1, delaySec);
					break;
				case "Ramp":
					status = auto.rampAuto(delaySec);
					break;
				case "Center":
					status = auto.centerAuto(isRedAlliance.getBoolean(true), 1, delaySec);
					break;
				default:
					status = Robot.DONE;
					break;
			}
    	}
		drive.updateOdometry();
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
		ledControl();
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
		drive.initTestDrivePower();
	}

	@Override
	/**
	 * testPeriodic()
	 * Runs constantly during test
	 */
	public void testPeriodic() {
		//drive.testEncoders();
		//drive.testWheelPower();
		//drive.periodicTestDrivePower();
		//drive.balanceRamp();
		drive.testGyro();
	}

	/**
	 * Controls the wheels in TeleOp
	 */
	private void wheelControl() {
		// Gets Joystick Values
		double forwardSpeed = controls.getForwardSpeed();
		double strafeSpeed  = controls.getStrafeSpeed();
		double rotateSpeed  = controls.getRotateSpeed();

		drive.teleopDrive(forwardSpeed, strafeSpeed, rotateSpeed, false);

		drive.updateOdometry();
		drive.testGyro();
	}

	private void ledControl() {
		boolean cone = controls.getCone();
		boolean cube = controls.getCube();

		long currentTime = System.currentTimeMillis();

		// Resetting timer for flashing what object we want
		if (cone) {
			coneFlashEnd = currentTime + (long) 5000;
			cubeFlashEnd = 0;
		}
		if (cube) {
			cubeFlashEnd = currentTime + (long) 5000;
			coneFlashEnd = 0;
		}

		// If we signaled for object less than 5 seconds ago, turn LED on half the time to create a flash
		if (currentTime < coneFlashEnd) {
			if ((coneFlashEnd - currentTime) % 400 < 200) {
				led.flashConeOn();
			}
			else {
				led.flashConeOff();
			}
		}
		else if (currentTime < cubeFlashEnd) {
			if ((cubeFlashEnd - currentTime) % 400 < 200) {
				led.flashCubeOn();
			}
			else {
				led.flashCubeOff();
			}
		}
		else {
			led.teamColors();
		}
		led.updateLED();
	}
}

// End of the Robot class