// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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
	PoseEstimation position;
	CustomTables   nTables;
	Controls       controls;
	Drive          drive;
	Auto           auto;
	LED            led;

	// Variables
	private int status = CONT;

	private long coneFlashEnd = 0;
	private long cubeFlashEnd = 0;

	// Auto path
	private static final String wallAuto   = "Wall";
	private static final String rampAuto   = "Ramp";
	private static final String centerAuto = "Center";
	private String m_autoSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();

	// Auto number of objects
	private int m_objectsToPlace;
	private final SendableChooser<Integer> m_objectChooser = new SendableChooser<>();

	// Auto Delay
	private long delaySec = 0;

	/**
	 * Constructor
	 */
	public Robot() {
		// Instance creation
		drive    = new Drive();
		controls = new Controls();
		position = new PoseEstimation(drive);
		auto     = new Auto(drive, position);

		// Instance getters
		led      = LED.getInstance();
		nTables  = CustomTables.getInstance();
	}

	@Override
	/**
	 * robotInit()
	 * Runs once when the robot is started.
	 */
	public void robotInit() {
		// Auto start location
		m_chooser.setDefaultOption(wallAuto, wallAuto);
		m_chooser.addOption(rampAuto, rampAuto);
		m_chooser.addOption(centerAuto, centerAuto);
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
	 * Runs every 20 miliseconds on the robot.
	 */
	public void robotPeriodic() {
		// Updates the PoseTrackers constantly
		position.updatePoseTrackers();
	}

	@Override
	/**
	 * autonomousInit()
	 * Runs once when the robot enters Autonomous mode.
	 */
	public void autonomousInit() {
		// Gets the auto delay 
		delaySec = (long)SmartDashboard.getNumber("Auto delay seconds", 0);

		// Choses start position
		m_autoSelected = m_chooser.getSelected();

		// Gets the number of objects to place in auto
		m_objectsToPlace = m_objectChooser.getSelected();

		// Resets the NavX Yaw
		drive.resetYaw();

		// Gets alliance color
		boolean isRed = nTables.getIsRedAlliance();

		// Creates the path starting locations
		Pose2d startPose = new Pose2d();

		// Sets the auto path to the selected one
		switch (m_autoSelected) {
			case wallAuto:
				if (isRed == true) {
					startPose = new Pose2d( 0, 0, new Rotation2d(0) );
				}
				else {
					startPose = new Pose2d( 0, 0, new Rotation2d(0) );
				}
				break;
			case rampAuto:
				if (isRed == true) {
					startPose = new Pose2d( 0, 0, new Rotation2d(0) );
				}
				else {
					startPose = new Pose2d( 0, 0, new Rotation2d(0) );
				}
				break;
			case centerAuto:
				if (isRed == true) {
					startPose = new Pose2d( 0, 0, new Rotation2d(0) );
				}
				else {
					startPose = new Pose2d( 0, 0, new Rotation2d(0) );
				}
				break;
			default:
				// Creates a default Pose2d
				startPose = new Pose2d();
				break;
		}

		// Sets the starting position
		drive.setGyroAngleZero(startPose.getRotation().getDegrees());
		position.resetPoseTrackers(startPose);
	}

	@Override
	/**
	 * autonomousPeriodic()
	 * Runs every 20 miliseconds during Autonomous.
	 */
	public void autonomousPeriodic() {
		// Runs the standatd autonomous switch statement
		if (status == Robot.CONT) {
			switch (m_autoSelected) {
				case "Wall":
					status = auto.wallAuto(nTables.getIsRedAlliance(), m_objectsToPlace, delaySec);
					break;
				case "Ramp":
					status = auto.rampAuto(nTables.getIsRedAlliance(), m_objectsToPlace, delaySec);
					break;
				case "Center":
					status = auto.centerAuto(nTables.getIsRedAlliance(), m_objectsToPlace, delaySec);
					break;
				default:
					status = Robot.DONE;
					break;
			}
    	}
	}

	@Override
	/**
	 * teleopInit()
	 * Runs once at the start of TeleOp.
	 */
	public void teleopInit() {
		// Resets the pose to the vision estimator's reading
		// position.resetPoseTrackers( position.getVisionPose() );
	}

	@Override
	/**
	 * teleopPeriodic()
	 * Runs ever 20 miliseconds during TeleOp.
	 */
	public void teleopPeriodic() {
		wheelControl();
		ledControl();
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
		// Inits the sliders
		status = Robot.CONT;
		drive.initWheelPowerTests();
	}

	@Override
	/**
	 * testPeriodic()
	 * Runs every 20 miliseconds during Test.
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

		boolean zeroYaw = controls.zeroYaw();

		if (zeroYaw) {
			drive.resetYaw();
			System.out.println("Zeroing yaw");
		}

		drive.teleopDrive(forwardSpeed, strafeSpeed, rotateSpeed, true);
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