// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Arm.AngleStates;
import frc.robot.Controls.ArmStates;

import edu.wpi.first.math.geometry.*;
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

	// Constants
	private final double ROTATE_SPEED_OFFSET = -0.16;

	// Variables
	private int count  = 0;
	private int status = CONT;
	private boolean firstTime = true;

	private long            coneFlashEnd              = 0;
	private long            cubeFlashEnd              = 0;
	private long            clawFlashEnd              = 0;
	public static ArmStates acceptedArmState;
	public static boolean   fromTop                   = false;
	private AngleStates     restStatus                = AngleStates.CONT;
	private int             armStatus                 = CONT;

	// Auto path
	private static final String wallAuto              = "Wall";
	private static final String rampAutoCone          = "Ramp Cone";
	private static final String rampAutoCube          = "Ramp Cube";
	private static final String rampAutoFullLeftCone  = "Full Ramp Left Cone";
	private static final String rampAutoFullRightCone = "Full Ramp Right Cone";
	private static final String centerAuto            = "Center";
	private String m_autoSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();

	// Auto Delay
	private long delaySec = 0;

	// Grabber states
	public static enum GrabberStates {
		EMPTY,
		HOLDING_CONE,
		HOLDING_CUBE,
		INTAKING_CONE,
		INTAKING_CUBE,
		EJECTING
	}
	public static GrabberStates grabberState = GrabberStates.EMPTY;

	/**
	 * Constructor
	 */
	public Robot() {
		// Instance creation
		arm      = new Arm();
		drive    = new Drive();
		controls = new Controls();
		position = new PoseEstimation(drive);
		auto     = new Auto(drive, position, arm, controls);

		// Instance getters
		led      = LED.getInstance();
		nTables  = CustomTables.getInstance();

		// Variables
		acceptedArmState = ArmStates.REST;
	}

	@Override
	/**
	 * robotInit()
	 * Runs once when the robot is started.
	 */
	public void robotInit() {
		// Auto start location
		m_chooser.setDefaultOption(rampAutoCone, rampAutoCone);
		m_chooser.addOption(rampAutoCube, rampAutoCube);
		m_chooser.addOption(rampAutoFullLeftCone, rampAutoFullLeftCone);
		m_chooser.addOption(rampAutoFullRightCone, rampAutoFullRightCone);
		m_chooser.addOption(wallAuto, wallAuto);
		m_chooser.addOption(centerAuto, centerAuto);
		SmartDashboard.putData("Auto choices", m_chooser);

		// Auto delay
		SmartDashboard.putNumber("Auto delay seconds", 0);

		// Sets the time for the Jetson
		if (firstTime == true) {
			firstTime = false;
			nTables.setTime(Timer.getFPGATimestamp());
		}

		// Sets the camera's width
		drive.setCamWidth(nTables.getCamWidth());
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
		// var pose = position.getOdometryPose();
		if (count % 10 == 0) {
			//System.out.println("X: " + Units.metersToInches(pose.getX()) + " Y: " + Units.metersToInches(pose.getY()) + " Yaw: " + pose.getRotation().getDegrees());
		}
		count++;
		SmartDashboard.putBoolean("Gyro Connected", drive.gyroConnected());
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

		// Gets alliance color
		boolean isRed = nTables.getIsRedAlliance();

		// Creates the path starting locations
		Pose2d startPose = new Pose2d();

		// Sets the auto path to the selected one
		switch (m_autoSelected) {
			case wallAuto:
				grabberState = GrabberStates.HOLDING_CONE;
				if (isRed == true) {
					startPose = new Pose2d(auto.WALL_RED_START, new Rotation2d(Math.PI));
				}
				else {
					startPose = new Pose2d(auto.WALL_BLUE_START, new Rotation2d(Math.PI));
				}
				break;
			case rampAutoCone:
				grabberState = GrabberStates.HOLDING_CONE;
				if (isRed == true) {
					startPose = new Pose2d(auto.RAMP_RED_START, new Rotation2d(Math.PI));
				}
				else {
					startPose = new Pose2d(auto.RAMP_BLUE_START, new Rotation2d(Math.PI));
				}
				break;
			case rampAutoCube:
				grabberState = GrabberStates.HOLDING_CUBE;
				if (isRed == true) {
					startPose = new Pose2d(auto.RAMP_RED_START, new Rotation2d(0));
				}
				else {
					startPose = new Pose2d(auto.RAMP_BLUE_START, new Rotation2d(0));
				}
				break;
			case rampAutoFullLeftCone:
				grabberState = GrabberStates.HOLDING_CONE;
				if (isRed == true) {
					startPose = new Pose2d(auto.RAMP_RED_START, new Rotation2d(Math.PI));
				}
				else {
					startPose = new Pose2d(auto.RAMP_BLUE_START, new Rotation2d(Math.PI));
				}
				break;
			case rampAutoFullRightCone:
				grabberState = GrabberStates.HOLDING_CONE;
				if (isRed == true) {
					startPose = new Pose2d(auto.RAMP_RED_START, new Rotation2d(Math.PI));
				}
				else {
					startPose = new Pose2d(auto.RAMP_BLUE_START, new Rotation2d(Math.PI));
				}
				break;
			case centerAuto:
				grabberState = GrabberStates.HOLDING_CONE;
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
					status = auto.wallAuto(nTables.getIsRedAlliance(), delaySec);
					break;
				case "Ramp Cone":
					status = auto.rampAuto(nTables.getIsRedAlliance(), delaySec);
					break;
				case "Ramp Cube":
					status = auto.rampAutoCube(nTables.getIsRedAlliance(), delaySec);
					break;	
				case "Full Ramp Left Cone":
					status = auto.rampAutoFull(nTables.getIsRedAlliance(), false, delaySec);
					break;
				case "Full Ramp Right Cone":
					status = auto.rampAutoFull(nTables.getIsRedAlliance(), true, delaySec);
					break;
				case "Center":
					status = auto.centerAuto(nTables.getIsRedAlliance(), delaySec);
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
		grabberControl();
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
		// if (status == Robot.CONT) {
		// 	double x = nTables.getGamePieceX();
		// 	status = drive.alignWithPiece(x, false);
		// }
		// else {
		// 	System.out.println("Done");
		// }
		//arm.startIntake();
		//arm.hold(3);
		arm.testAbsEncoders();
	}

	/**
	 * Controls the wheels in TeleOp
	 */
	private void wheelControl() {
		// Variables
		double  centerX           = nTables.getGamePieceX();
		double  numCones          = nTables.getNumCones();
		double  numCubes          = nTables.getNumCubes();
		boolean switchPressed     = controls.getLimitSwitch();
		boolean recentAprilTag    = position.recentAprilTag(); // Check in pose estimation if we read April Tag within last 5 seconds

		// Gets the Drive Values
		double  rotateSpeed       = controls.getRotateSpeed();
		double  strafeSpeed       = controls.getStrafeSpeed();
		double  forwardSpeed      = controls.getForwardSpeed();
		double  robotOriented     = controls.robotOrientedSpeed();
		boolean alignWithPiece    = controls.allignWithPiece();
		boolean precisionDrive    = controls.enablePrecisionDrive();

		// Gets the Manipulator values
		boolean zeroYaw           = controls.zeroYaw();
		boolean autoKill          = controls.autoKill();
		boolean lockWheels        = controls.lockWheels();

		position.updateVision = true;

		if (zeroYaw == true) {
			drive.resetYaw();
		}

		if (lockWheels == true) {
			drive.crossWheels();
			drive.resetDriveToPoints();
		}
		else if (alignWithPiece == true) {
			if (switchPressed == false) {
				drive.alignWithPiece(centerX, true);
			}
			drive.resetDriveToPoints();
		}
		else if (robotOriented != 0) {
			drive.teleopDrive(robotOriented, 0, rotateSpeed, false);
		}
		else if (autoKill == true || (!recentAprilTag)) {
			if (precisionDrive)  {
				drive.teleopDrive(forwardSpeed / 3, strafeSpeed / 3, rotateSpeed / 3, true);
			}
			else {
				// Calculated line of best fit for relationship between rotate speed and drift angle
				double angleTransform = ROTATE_SPEED_OFFSET * rotateSpeed;
				Translation2d velocity = new Translation2d(forwardSpeed, strafeSpeed);
				Translation2d newVelocity = velocity.rotateBy(new Rotation2d(angleTransform));

				double newXVel = newVelocity.getX();
				double newYVel = newVelocity.getY();

				drive.teleopDrive(newXVel, newYVel, rotateSpeed, true);
			}
			drive.resetDriveToPoints();
		}
	}

	private void armControl() {
		// Add the grabber controls
		boolean   autoKill         = controls.autoKill();
		double    manualWristPower = controls.getManualWristPower();
		ArmStates inputArmState    = controls.getArmState();

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
					armStatus = auto.armToMidCube();
					fromTop = false;
				}
				else if (acceptedArmState == ArmStates.TOP_CONE) {
					armStatus = auto.armToTopConeTeleop();
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
				else if (acceptedArmState == ArmStates.CHUTE_CONE) {
					// armStatus stays at CONT because PID can stay on when we are close
					auto.armToChuteCone();
					fromTop = false;
				}
				else if (acceptedArmState == ArmStates.CHUTE_CUBE) {
					// armStatus stays at CONT because PID can stay on when we are close
					auto.armToChuteCube();
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
	}

	private void grabberControl() {
		boolean intakeCube  = controls.getLeftBumper();  // Checks if we are holding L bumper to intake cube
		boolean intakeCone  = controls.getRightBumper(); // Checks if we are holding R bumper to intake cone
		boolean eject       = controls.getEject();       // Checks if we are holding R trigger to eject our object
		boolean launch      = controls.getLaunch();      // Checks if we are holding L trigger to shoot our object
		boolean limitButton = controls.getLimitSwitch(); // Checks if button on back of claw is hit
		boolean intakeCubePressed = controls.getLeftBumperPressed();
		boolean intakeConePressed = controls.getRightBumperPressed();

		if (grabberState == GrabberStates.EMPTY) {
			// Action
			arm.stopIntake();
			arm.openClaw();

			// Next states
			if (intakeCone) {
				grabberState = GrabberStates.INTAKING_CONE;
			}
			else if (intakeCube) {
				grabberState = GrabberStates.INTAKING_CUBE;
			}
			else if (eject || launch) {
				grabberState = GrabberStates.EJECTING;
			}
		}
		else if (grabberState == GrabberStates.INTAKING_CONE) {
			// Action
			arm.startIntake();
			arm.openClaw();

			// Next states
			if (limitButton || (!intakeCone)) {
				grabberState = GrabberStates.HOLDING_CONE;
				clawFlashEnd = System.currentTimeMillis() + 1000;
			}
		}
		else if (grabberState == GrabberStates.INTAKING_CUBE) {
			// Action
			arm.startIntake();
			arm.openClaw();

			// Next states
			if (limitButton || (!intakeCube)) {
				grabberState = GrabberStates.HOLDING_CUBE;
				clawFlashEnd = System.currentTimeMillis() + 1000;
			}
		}
		else if (grabberState == GrabberStates.HOLDING_CONE || grabberState == GrabberStates.HOLDING_CUBE) {
			// Action
			arm.stopIntake();
			arm.closeClaw();

			// Next states
			if (eject || launch) {
				grabberState = GrabberStates.EJECTING;
			}
			// Can switch objects in case of mistake while holding
			else if (intakeConePressed) {
				grabberState = GrabberStates.INTAKING_CONE;
			}
			else if (intakeCubePressed) {
				grabberState = GrabberStates.INTAKING_CUBE;
			}
		}
		else if (grabberState == GrabberStates.EJECTING) {
			// Actions
			arm.openClaw();

			if (eject) {
				arm.startEject();
			}
			else if (launch) {
				arm.startLaunch();
			}
			// Next state
			else {
				grabberState = GrabberStates.EMPTY;
			}
		}

	}

	private void ledControl() {
		boolean cone = controls.getCone();
		boolean cube = controls.getCube();
		boolean lock = controls.lockWheels();
		
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
		
		if (lock) {
			led.party();
		}
		// LED's will flash green when claw closed
		else if (currentTime < clawFlashEnd) {
			led.clawClosed();
		}
		// If we signaled for object less than 5 seconds ago, turn LED on alternate flashing colors
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
