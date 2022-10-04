package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.networktables.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The class that runs eveything else
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
  Drive    drive;
  Controls controls;

  // Variables
  private int status = CONT;

  // Enumeration for manual or automatic control
  public static enum DriveMode {
    MANUAL;
  }
  private DriveMode driveMode = DriveMode.MANUAL;

  // Auto path
  private static final String kCenterAuto = "Center";
  private static final String kWallAuto   = "Wall";
  private static final String kHangarAuto = "Hangar";
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
    m_chooser.setDefaultOption("Wall Auto", kWallAuto);
    m_chooser.addOption("Center Auto", kCenterAuto);
    m_chooser.addOption("Hangar Auto", kHangarAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // Auto delay
    SmartDashboard.putNumber("Auto delay seconds", 0);
  }

  @Override
  /**
   * robotPeriodic()
   * Always runs on the robot
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
    System.out.println("Auto selected: " + m_autoSelected);

    delaySec = (int)SmartDashboard.getNumber("Auto delay seconds", 0);

    // Resets the gyro
    Drive.ahrs.zeroYaw();
  }

  @Override
  /**
   * autonomousPeriodic()
   * Runs constantly during Autonomous
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
  }

  @Override
  /**
   * teleopPeriodic()
   * Runs constantly during TeleOp
   */
  public void teleopPeriodic() {
    wheelControl();
  }

  @Override
  /**
   * disabledInit()
   */
  public void disabledInit() {    
  }

  @Override
  /**
   * disabledPeriodic()
   * Shouldn't do anything
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
  }

  @Override
  /**
   * testPeriodic()
   * Runs constantly during test
   */
  public void testPeriodic() {
  }

  /**
   * Controls the wheels in TeleOp
   */
  private void wheelControl() {
    // Gets Joystick Values
    double driveX               = controls.getDriveX();
    double driveY               = controls.getDriveY();
    double rotatePower          = controls.getRotatePower();

    // Manual driving
    if (driveMode == DriveMode.MANUAL) {
    // Drives if we are out of dead zone
    if ((Math.abs(driveX) > 0.05) ||
        (Math.abs(driveY) > 0.05) || 
        (Math.abs(rotatePower) > 0.01)) {
        drive.teleopSwerve(driveX, driveY, rotatePower, false, true);
    }
    else {
        // Robot is in dead zone, doesn't drive
        drive.stopWheels();
    }
    } 

    // Limelight targeting
    else {
    //
    }
  }

  /**
   * Determines if we are on the red alliance
   * @return isRed
   */
  private boolean getRedAlliance() {
    // Gets and returns if we are red from the FMS
    boolean isRed = isRedAlliance.getBoolean(false);
    return isRed;
  }

}

// End of the Robot class