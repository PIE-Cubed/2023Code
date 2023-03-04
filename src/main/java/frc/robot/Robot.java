// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Arm.AngleStates;
import frc.robot.Controls.ArmStates;
import frc.robot.Controls.Objects;

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
	Arm            arm;
	PoseEstimation position;
	CustomTables   nTables;
	Controls       controls;
	Drive          drive;
	Auto           auto;
	LED            led;

	// Variables
	private int status = CONT;

	private long      coneFlashEnd = 0;
	private long      cubeFlashEnd = 0;
	private Pose2d    previousPlacementLocation;
	private int       placementStatus = Robot.CONT;
	private ArmStates acceptedArmState;

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
		position = new PoseEstimation(drive);

		controls = new Controls();
		arm      = new Arm();
		auto     = new Auto(drive, position, arm);

		// Instance getters
		led      = LED.getInstance();
		nTables  = CustomTables.getInstance();

		previousPlacementLocation = new Pose2d();
		acceptedArmState          = ArmStates.REST;
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
					startPose = new Pose2d(auto.WALL_RED_START, new Rotation2d(-Math.PI/2));
				}
				else {
					startPose = new Pose2d(auto.WALL_BLUE_START, new Rotation2d(-Math.PI/2));
				}
				break;
			case rampAuto:
				if (isRed == true) {
					startPose = new Pose2d(auto.RAMP_RED_START, new Rotation2d(-Math.PI/2));
				}
				else {
					startPose = new Pose2d(auto.RAMP_BLUE_START, new Rotation2d(-Math.PI/2));
				}
				break;
			case centerAuto:
				if (isRed == true) {
					startPose = new Pose2d(auto.CENTER_RED_START, new Rotation2d(-Math.PI/2));
				}
				else {
					startPose = new Pose2d(auto.CENTER_BLUE_START, new Rotation2d(-Math.PI/2));
				}
				break;
			default:
				// Creates a default Pose2d
				startPose = new Pose2d();
				break;
		}

		// Sets the starting position
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
		//drive.resetYaw();
		//drive.setGyroAngleZero(90);
		//position.resetPoseTrackers(new Pose2d(1.767, 1.067, new Rotation2d(Math.PI)) );
	}

	@Override
	/**
	 * teleopPeriodic()
	 * Runs ever 20 miliseconds during TeleOp.
	 */
	public void teleopPeriodic() {
		armControl();
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
		//drive.testGyro();
		//arm.testMiddlePower();
		arm.testAbsEncoders();
		//AngleStates status = arm.jointToAngle(1, Math.PI/2);
		//System.out.println(status);
	}

	/**
	 * Controls the wheels in TeleOp
	 */
	private void wheelControl() {
		// Gets Joystick Values
		Pose2d  currentLocation   = position.getVisionPose();
		double  forwardSpeed      = controls.getForwardSpeed();
		double  strafeSpeed       = controls.getStrafeSpeed();
		double  rotateSpeed       = controls.getRotateSpeed();
		Pose2d  placementLocation = controls.getPlacementLocation(currentLocation.getY(), nTables.getIsRedAlliance());
		boolean autoKill          = controls.autoKill();

		if (placementLocation == null) {
			drive.teleopDrive(forwardSpeed, strafeSpeed, rotateSpeed, true);
		}
		else if (autoKill) {
			drive.teleopDrive(forwardSpeed, strafeSpeed, rotateSpeed, true);
			drive.resetDriveToPoints();
		}
		else {
			// Resets placement location if we change locations
			if (!placementLocation.equals(previousPlacementLocation))	{
				placementStatus = Robot.CONT;
				drive.resetDriveToPoints();
			}	
			previousPlacementLocation = placementLocation;

			// Stops trying to position once status is done
			if (placementStatus == Robot.CONT) {
				placementStatus = drive.autoDriveToPoints(new Pose2d[]{placementLocation}, currentLocation);
			}	
		}
	}

	private void armControl() {
		// Add the grabber controls
		Objects   currentObject    = controls.getClawState();
		double    manualWristPower = 0; //controls.getManualWristPower();
		ArmStates inputArmState    = controls.getArmState();
		boolean   autoKill         = controls.autoKill();

		// Manual wrist control overrides automatic control
		if (manualWristPower != 0) {
			arm.powerEnd(manualWristPower);
		}
		else {
			AngleStates status;

			// Bring arm through rest position to our target position
			if (acceptedArmState == ArmStates.REST) {
				// Movement
				status = auto.armToRestPosition();

				// Conditions to change arm state - close to resting and receives different target state
				if ((status == AngleStates.DONE || status == AngleStates.CLOSE) && inputArmState != ArmStates.REST) {
					acceptedArmState = inputArmState;
					auto.resetArmRoutines();
				}
			}
			else {
				int placeStatus = CONT;

				// Movement
				if (acceptedArmState == ArmStates.GRAB) {
					auto.armToGrabPosition();
				}
				else if (acceptedArmState == ArmStates.MID_CONE) {
					auto.armToMidPosition(arm.MID_CONE_ANGLES);
				}
				else if (acceptedArmState == ArmStates.MID_CUBE) {
					auto.armToMidPosition(arm.MID_CUBE_ANGLES);
				}
				else if (acceptedArmState == ArmStates.TOP_CONE) {
					auto.armToTopCone();
				}
				else if (acceptedArmState == ArmStates.TOP_CUBE) {
					auto.armToTopCube();
				}
				else if (acceptedArmState == ArmStates.SHELF) {
					auto.armToShelf();
				}
				// No valid arm state - go to rest
				else {
					Controls.armState = ArmStates.REST;
					acceptedArmState = ArmStates.REST;
					auto.resetArmRoutines();
				}

				// Conditions to change to rest state - receive rest input or finish placing object or autoKill
				if (inputArmState == ArmStates.REST || placeStatus == DONE || autoKill) {
					Controls.armState = ArmStates.REST;
					acceptedArmState = ArmStates.REST;
					auto.resetArmRoutines();
				}
			}
		}

		if (currentObject == Objects.EMPTY) {
			arm.openClaw();
		}
		else {
			arm.closeClaw();
		}
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