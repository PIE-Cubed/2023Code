// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auto.*;
//import frc.robot.commands.*;
import frc.robot.commands.CommandGroups.*;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
	LED            led;

	// Command creation
    private Command autoCommand;
	private Command testCommand;

	// Auto path
	private static final String leftAuto   = "Left";
	private static final String rightAuto  = "Right";
	private static final String middleAuto = "Middle";
	private String m_autoSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();

	// Auto Delay
	private int delaySec = 0;

	/**
	 * Constructor
	 */
	public Robot() {
		// Instance creation
		drive    = new Drive();
		controls = new Controls();
		position = new PoseEstimation(drive);

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
		m_chooser.addOption("Left", leftAuto);
		m_chooser.addOption("Right", rightAuto);
		m_chooser.setDefaultOption("Middle", middleAuto);
		SmartDashboard.putData("Auto choices", m_chooser);

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
		//position.updatePoseTrackers();

		// Runs the CommandScheduler
		CommandScheduler.getInstance().run();
	}

	@Override
	/**
	 * autonomousInit()
	 * Runs once when the robot enters Autonomous mode.
	 */
	public void autonomousInit() {
		// Choses start position
		m_autoSelected = m_chooser.getSelected();
		System.out.println("Auto selected: " + m_autoSelected);

		// Gets the auto delay
		delaySec = (int)SmartDashboard.getNumber("Auto delay seconds", 0);

		// Creates the path starting locations
		Pose2d defaultStart = new Pose2d();

		// Selects an auto command to run
		switch (m_autoSelected) {
			default:
				position.resetPoseTrackers(defaultStart);
				autoCommand = new BasicAuto(drive, position, delaySec);
				break;
		}

		// Schedules autoCommand to run
        autoCommand.schedule();
	}

	@Override
	/**
	 * autonomousPeriodic()
	 * Runs every 20 miliseconds during Autonomous.
	 */
	public void autonomousPeriodic() {
		// Since the commands are sequential, nothing needs to be here
	}

	@Override
	/**
	 * teleopInit()
	 * Runs once at the start of TeleOp.
	 */
	public void teleopInit() {
		// Makes sure that the autonomous stops running when teleop starts
        if (autoCommand != null) {
            autoCommand.cancel();
        }

		//
		position.resetPoseTrackers(new Pose2d(new Translation2d(), new Rotation2d(-Math.PI/2)));
	}

	@Override
	/**
	 * teleopPeriodic()
	 * Runs ever 20 miliseconds during TeleOp.
	 */
	public void teleopPeriodic() {
		wheelControl();
		position.updatePoseTrackers();

		Pose2d thing = position.getVisionPose();
		// System.out.println(
		// 	"X Position: " + Units.metersToInches(thing.getTranslation().getX()) +
		// 	" Y Position: " + Units.metersToInches(thing.getTranslation().getY())
		// );
	}

	@Override
	/**
	 * disabledInit()
	 * Runs once when the robot enteres Disabled mode.
	 */
	public void disabledInit() {
		// Cancels all commands
		CommandScheduler.getInstance().cancelAll(); 
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
		drive.initWheelPowerTests();

		// Creates the test command
		testCommand = new TestModules(drive);

		// Runs the test command
		testCommand.schedule();
	}

	@Override
	/**
	 * testPeriodic()
	 * Runs every 20 miliseconds during Test.
	 */
	public void testPeriodic() {
		// drive.testEncoders();
		// drive.testWheelPowers();
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
	}
}

// End of the Robot class