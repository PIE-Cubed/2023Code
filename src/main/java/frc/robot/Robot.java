// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Arm.AngleStates;
import frc.robot.Controls.Objects;
import frc.robot.Controls.ArmStates;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.TimedRobot;
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
	Arm            arm;

	// Variables
	private int count  = 0;
	private int status = CONT;
	private boolean firstTime = true;

	private long            coneFlashEnd              = 0;
	private long            cubeFlashEnd              = 0;
	private long            aprilTagStart             = 0;
	private long            failAprilTagStart         = 0;
	private boolean         placementPositionError    = false;
	private Pose2d          previousPlacementLocation;
	private boolean         previousRecentAprilTag    = false;
	private int             placementStatus           = Robot.CONT;
	public static ArmStates acceptedArmState;
	public static boolean   fromTop                   = false;
	private AngleStates     restStatus                = AngleStates.CONT;
	private int             armStatus                 = CONT;

	// Auto path
	private static final String wallAuto     = "Wall";
	private static final String rampAuto     = "Ramp";
	private static final String rampAutoFull = "Full Ramp";
	private static final String centerAuto   = "Center";
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
		arm      = new Arm();
		drive    = new Drive();
		controls = new Controls();
		position = new PoseEstimation(drive);
		auto     = new Auto(drive, position, arm);

		// Instance getters
		led      = LED.getInstance();
		nTables  = CustomTables.getInstance();

		// Variables
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
		m_chooser.setDefaultOption(rampAuto, rampAuto);
		m_chooser.addOption(rampAutoFull, rampAutoFull);
		m_chooser.addOption(wallAuto, wallAuto);
		m_chooser.addOption(centerAuto, centerAuto);
		SmartDashboard.putData("Auto choices", m_chooser);

		// Auto objects placed
		m_objectChooser.setDefaultOption("1", 1);
		m_objectChooser.addOption("2", 2);
		m_objectChooser.addOption("3", 3);
		SmartDashboard.putData("Auto objects placed", m_objectChooser);

		// Auto delay
		SmartDashboard.putNumber("Auto delay seconds", 0);

		// Sets the time for the Jetson
		if (firstTime == true) {
			firstTime = false;
			nTables.setTime(Timer.getFPGATimestamp());
		}
	}

	@Override
	/**
	 * robotPeriodic()
	 * Runs every 20 miliseconds on the robot.
	 */
	public void robotPeriodic() {
		// Updates the PoseTrackers constantly
		position.updatePoseTrackers();

		// Prints the pose
		var pose = position.getOdometryPose();
		if (count % 10 == 0) {
			//System.out.println("X: " + Units.metersToInches(pose.getX()) + " Y: " + Units.metersToInches(pose.getY()) + " Yaw: " + pose.getRotation().getDegrees());
		}
		count++;
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
					startPose = new Pose2d(auto.WALL_RED_START, new Rotation2d(Math.PI));
				}
				else {
					startPose = new Pose2d(auto.WALL_BLUE_START, new Rotation2d(Math.PI));
				}
				break;
			case rampAuto:
				if (isRed == true) {
					startPose = new Pose2d(auto.RAMP_RED_START, new Rotation2d(Math.PI));
				}
				else {
					startPose = new Pose2d(auto.RAMP_BLUE_START, new Rotation2d(Math.PI));
				}
				break;
			case rampAutoFull:
				if (isRed == true) {
					startPose = new Pose2d(auto.RAMP_RED_START, new Rotation2d(Math.PI));
				}
				else {
					startPose = new Pose2d(auto.RAMP_BLUE_START, new Rotation2d(Math.PI));
				}
				break;
			case centerAuto:
				if (isRed == true) {
					startPose = new Pose2d(auto.CENTER_RED_START, new Rotation2d(Math.PI));
				}
				else {
					startPose = new Pose2d(auto.CENTER_BLUE_START, new Rotation2d(Math.PI));
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
				case "Full Ramp":
					status = auto.rampAutoFull(nTables.getIsRedAlliance(), m_objectsToPlace, delaySec);
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
		// Nothing yet...
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
		// Reset status
		status = Robot.CONT;
	}

	@Override
	/**
	 * testPeriodic()
	 * Runs every 20 miliseconds during Test.
	 */
	public void testPeriodic() {
		System.out.println("Button:" + arm.limitButtonPressed());
		Pose2d pose = position.getPose();
 
		Pose2d[] points = {
			new Pose2d(new Translation2d(Units.inchesToMeters(110), Units.inchesToMeters(85)), new Rotation2d(Math.PI))
		};

		if (status == Robot.CONT) {
			status = drive.autoDriveToPoints(points, pose);
		}
		else {
			drive.stopWheels();
		}
	}

	/**
	 * Controls the wheels in TeleOp
	 */
	private void wheelControl() {
		// Gets Joystick Values
		Pose2d  currentLocation   = position.getPose();
		double  forwardSpeed      = controls.getForwardSpeed();
		double  strafeSpeed       = controls.getStrafeSpeed();
		double  rotateSpeed       = controls.getRotateSpeed();
		Pose2d  placementLocation = controls.getPlacementLocation(currentLocation.getY(), nTables.getIsRedAlliance());
		boolean autoKill          = controls.autoKill();
		boolean lockWheels        = controls.lockWheels();
		boolean zeroYaw           = controls.zeroYaw();
		boolean precisionDrive    = controls.enablePrecisionDrive();
		boolean recentAprilTag    = position.recentAprilTag(); // Check in pose estimation if we read April Tag within last 5 seconds
		
		position.updateVision = true;

		if (zeroYaw) {
			drive.resetYaw();
		}

		if (lockWheels) {
			drive.crossWheels();
			drive.resetDriveToPoints();
			previousPlacementLocation = null;
			placementPositionError = false;
		}
		else if (placementLocation == null || autoKill || (!recentAprilTag)) {
			if (precisionDrive)  {
				drive.teleopDrive(forwardSpeed / 3, strafeSpeed / 3, rotateSpeed / 3, true);
			}
			else {
				drive.teleopDrive(forwardSpeed, strafeSpeed, rotateSpeed, true);
			}
			drive.resetDriveToPoints();
			previousPlacementLocation = null;
				
			// Red LED's if we tried to go to placement position without an April Tag reading
			if (placementLocation != null && (!recentAprilTag)) {
				placementPositionError = true;
			}
			else {
				placementPositionError = false;
			}
		}
		else {
			placementPositionError = false;

			// Resets placement location if we change locations
			if (!placementLocation.equals(previousPlacementLocation))	{
				placementStatus = Robot.CONT;
				drive.resetDriveToPoints();
			}	
			previousPlacementLocation = placementLocation;

			// Stops trying to position once status is done
			if (placementStatus == Robot.CONT) {
				//placementStatus = drive.atDrive(placementLocation.getY(), currentLocation);
				drive.atDrive(placementLocation.getY(), currentLocation);
				position.updateVision = false;
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
			// Bring arm through rest position to our target position
			if (acceptedArmState == ArmStates.REST) {
				// Movement
				if (restStatus == AngleStates.DONE) {
					arm.stopArm();
				}
				else {
					restStatus = auto.armToRestPosition(fromTop);
				}

				// Conditions to change arm state - close to resting and receives different target state
				if ((restStatus == AngleStates.DONE || restStatus == AngleStates.CLOSE) && inputArmState != ArmStates.REST) {
					acceptedArmState = inputArmState;
					armStatus = CONT;
					auto.resetArmRoutines();
				}
			}
			else {
				// Movement
				if (armStatus == DONE) {
					arm.hold(1);
					arm.hold(2);
					arm.hold(3);
				}
				else if (acceptedArmState == ArmStates.GRAB) {
					// armStatus stays at CONT because PID can stay on when we are close
					auto.armToGrabPosition();
					fromTop = false;
				}
				else if (acceptedArmState == ArmStates.MID_CONE) {
					armStatus = auto.armToMidPosition(Arm.MID_CONE_ANGLES);
					fromTop = false;
				}
				else if (acceptedArmState == ArmStates.MID_CUBE) {
					armStatus = auto.armToMidPosition(Arm.MID_CUBE_ANGLES);
					fromTop = false;
				}
				else if (acceptedArmState == ArmStates.TOP_CONE) {
					armStatus = auto.armToTopCone();
					fromTop = true;
				}
				else if (acceptedArmState == ArmStates.TOP_CUBE) {
					armStatus = auto.armToTopCube();
					fromTop = false;
				}
				else if (acceptedArmState == ArmStates.SHELF) {
					armStatus = auto.armToShelf();
					fromTop = false;
				}
				else if (acceptedArmState == ArmStates.CHUTE) {
					// armStatus stays at CONT because PID can stay on when we are close
					auto.armToChute();
					fromTop = false;
				}
				
				// Conditions to change to rest state - receive rest input or finish placing object or autoKill
				if (inputArmState == ArmStates.REST || autoKill) {
					Controls.armState = ArmStates.REST;
					acceptedArmState = ArmStates.REST;
					restStatus = AngleStates.CONT;
					auto.resetArmRoutines();
				}
			}
		}

		if (arm.limitButtonPressed()) {
			Controls.currentObject = Objects.CONE;
			currentObject = Objects.CONE;
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
		boolean lock = controls.lockWheels();
		
		boolean recentAprilTag = position.recentAprilTag(); // Check in pose estimation if we read April Tag within last 5 seconds

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
		
		// Just picked up April Tag
		if (previousRecentAprilTag == false && recentAprilTag == true) {
			aprilTagStart = currentTime;
		}
		// Lost April Tag
		else if (previousRecentAprilTag == true && recentAprilTag == false) {
			failAprilTagStart = currentTime;
		}
		previousRecentAprilTag = recentAprilTag;
		
		// Highest priority - tried to go to placement position without an April Tag reading
		// Will only happen while holding button to go to position
		if (placementPositionError) {
			led.noAprilTag();
		}
		else if (lock) {
			led.party();
		}
		// LED's will flash green every 5 seconds if we had a recent April Tag reading
		else if (recentAprilTag && (currentTime - aprilTagStart) % 5000 < 400) {
			led.aprilTagVisible();
		}
		// LED's will flash red if we lose April Tag reading
		else if (currentTime < failAprilTagStart + 400) {
			led.noAprilTag();
		}
		// If we signaled for object less than 5 seconds ago, turn LED on half the time to create a flash
		else if (currentTime < coneFlashEnd) {
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
