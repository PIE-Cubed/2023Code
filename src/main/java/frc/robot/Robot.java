// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.ShootLocation;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.CANSparkMax.IdleMode;

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
    LedLights     led;
    PDH           pdh;
    Drive         drive;
    Grabber       grabber;
    Shooter       shooter;
    Climber       climber;
    Controls      controls;
    CustomTables  customTables;

    // Command creation
    private Command autoCommand;

    // Variables
    private int targetStatus        = Robot.CONT;
    private boolean lowShotEnabled  = false;

    // Enumeration for manual or automatic control
    public static enum DriveMode {
        MANUAL,
        LIMELIGHT_TARGETING,
        LIMELIGHT_TARGETED,
        CARGO_TARGETING,
        CARGO_TARGETED;
    }
    private DriveMode driveMode = DriveMode.MANUAL;

    // Auto path
    private static final String kCenterAuto = "Center";
    private static final String kHangarAuto = "Hangar";
    private static final String kWallAuto   = "Wall";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // Number of balls
    private static final int kTwoBall   = 2;
    private static final int kThreeBall = 3;
    private int m_numBalls;
    private final SendableChooser<Integer> m_numBallsChooser = new SendableChooser<>();

    // Auto Delay
    private int delaySec = 0;

    /**
     * The constructor for the Robot class
     */
    public Robot() {
        // Instance creation
        led           = new LedLights();
        pdh           = new PDH();
        drive         = new Drive();
        grabber       = new Grabber();
        controls      = new Controls();
        climber       = new Climber(led);
        shooter       = new Shooter();
    }

    @Override
    /**
     * robotInit()
     * Runs once when the robot is started
     */
    public void robotInit() {
        // Auto start location
        m_chooser.addOption("Center Auto", kCenterAuto);
        m_chooser.addOption("Hangar Auto", kHangarAuto);
        m_chooser.setDefaultOption("Wall Auto", kWallAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        // Auto delay
        SmartDashboard.putNumber("Auto delay seconds", 0);

        // Number of Balls to grab
        m_numBallsChooser.setDefaultOption("2 ball", kTwoBall);
        m_numBallsChooser.addOption("3 ball", kThreeBall);
        SmartDashboard.putData("Number of Balls", m_numBallsChooser);
    }

    @Override
    /**
     * robotPeriodic()
     * Always runs on the robot
     */
    public void robotPeriodic() {
        // Passes our color to the lights
        led.defaultMode( getRedAlliance() );

        // Constantly updates the swerve odometry
        drive.updateAllPoseTrackers();

        // Constantly updates the time for the Jetson
        CustomTables.setTimeSec();

        // Constantly updates the gyro for the Jetson
        CustomTables.setGyroYaw(drive.getHeading());

        // Constantly updates the module positions for the Jetson
        SwerveModulePosition[] allPos = drive.getModulePositons();
        CustomTables.setFLState(allPos[0].distanceMeters, allPos[0].angle.getDegrees());
        CustomTables.setRLState(allPos[1].distanceMeters, allPos[1].angle.getDegrees());
        CustomTables.setFRState(allPos[2].distanceMeters, allPos[2].angle.getDegrees());
        CustomTables.setRRState(allPos[3].distanceMeters, allPos[3].angle.getDegrees());

        // Runs the CommandScheduler
        CommandScheduler.getInstance().run();
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

        m_numBalls = m_numBallsChooser.getSelected();
        System.out.println("Auto path: " + m_numBalls);

        delaySec = (int)SmartDashboard.getNumber("Auto delay seconds", 0);

        // Inits the ledlights for auto
        led.autoInit();

        // Selects an auto command to run
        switch (m_autoSelected) {
            case kCenterAuto:
                autoCommand = new Center(drive, shooter, grabber, led, delaySec);
                drive.resetPoseTrackers(null);
                break;
            case kHangarAuto:
                autoCommand = new Hangar(drive, shooter, grabber, led, delaySec);
                drive.resetPoseTrackers(null);
                break;
            case kWallAuto:
                autoCommand = new Wall(drive, shooter, grabber, led, delaySec);
                drive.resetPoseTrackers(null);
                break;
            default:
                autoCommand = new Wall(drive, shooter, grabber, led, delaySec);
                drive.resetPoseTrackers(null);
                break;
        }

        // Schedules autoCommand to run
        autoCommand.schedule();
    }

    @Override
    /**
     * autonomousPeriodic()
     * Runs constantly during Autonomous
     */
    public void autonomousPeriodic() {
        // Since the commands are sequential, nothing needs to be here
    }

    @Override
    /**
     * teleopInit()
     * Runs once at the start of TeleOp
     */
    public void teleopInit() {
        // Inits the led lights
        led.teleopInit();

        // Sets the climber motor to brake
        climber.setClimberIdleMode(IdleMode.kBrake);

        // Makes sure that the autonomous stops running when teleop starts
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    @Override
    /**
     * teleopPeriodic()
     * Runs constantly during TeleOp
     */
    public void teleopPeriodic() {
        wheelControl();
        ballControl();
        climberControl();
    }

    @Override
    /**
     * disabledInit()
     */
    public void disabledInit() {
        // Sets the climber motors to coast
        climber.setClimberIdleMode(IdleMode.kCoast);

        // Sets the Led's to team colors when disabled
        led.teamColors();
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
        //
        Command chaseTag = new ChaseTag(drive, drive::getVisionPose);
        chaseTag.schedule();
    }

    @Override
    /**
     * testPeriodic()
     * Runs constantly during test
     */
    public void testPeriodic() {
        //shooter.testShootMotor(SmartDashboard.getNumber("Shoot Power", 0.00));
        //drive.testWheelAngle();
    }

    /**
     * Controls the wheels in TeleOp
     */
    private void wheelControl() {
        // Gets Joystick Values
        double driveX               = controls.getDriveX();
        double driveY               = controls.getDriveY();
        double rotatePower          = controls.getRotatePower();
        boolean fieldDriveEnabled   = controls.toggleFieldDrive();
        boolean enableCargoTracking = controls.enableCargoTracking();
        ShootLocation shootLocation = controls.getShootLocation();

        // Kills all automatic funcitons (Start on the Xbox controller)
        boolean autokill            = controls.autoKill();

        if (autokill == true) {
            driveMode = DriveMode.MANUAL;
        } 

        // Manual driving
        if (driveMode == DriveMode.MANUAL) {
            // Drives if we are out of dead zone
            if ((Math.abs(driveX) > 0.05) ||
                (Math.abs(driveY) > 0.05) || 
                (Math.abs(rotatePower) > 0.01)) {
                drive.teleOpDrive(driveX, driveY, rotatePower, fieldDriveEnabled);
            }
            else {
                // Robot is in dead zone, doesn't drive
                drive.stopWheels();
            }

            // Sets a variable that is used later in ShooterControl()
            if (shootLocation == ShootLocation.LOW_SHOT) {
                lowShotEnabled = true;
            }
            else {
                lowShotEnabled = false;
            }

            // Exit conditions
            if ((shootLocation == ShootLocation.HIGH_SHOT) || (shootLocation == ShootLocation.LAUNCH_PAD)) {
                driveMode = DriveMode.LIMELIGHT_TARGETING;
            }

            // Exit conditions
            if (enableCargoTracking == true) {
                driveMode = DriveMode.CARGO_TARGETING;
            }
        } 
        // Limelight targeting
        else if (driveMode == DriveMode.LIMELIGHT_TARGETING) {
            if (shootLocation == ShootLocation.OFF || shootLocation == ShootLocation.LOW_SHOT) {
                driveMode = DriveMode.MANUAL;
            }
            else {
                //
            }

            // Exit conditions
            if (targetStatus == Robot.DONE) {
                driveMode = DriveMode.LIMELIGHT_TARGETED;
            }
            else if (targetStatus == Robot.FAIL) {
                driveMode = DriveMode.MANUAL;
            }
        }
        // Limelight targeted
        else if (driveMode == DriveMode.LIMELIGHT_TARGETED) {
            // Does nothing until trigger is released (exit condition)
            if (shootLocation == ShootLocation.OFF || shootLocation == ShootLocation.LOW_SHOT) {
                driveMode = DriveMode.MANUAL;
            }
        }
        // Raspberry Pi Targeting
        else if (driveMode == DriveMode.CARGO_TARGETING) {
            int cargoStatus = Robot.DONE;

            if (cargoStatus == Robot.DONE) {
                driveMode = DriveMode.CARGO_TARGETED;
            }
            else if (cargoStatus == Robot.FAIL) {
                driveMode = DriveMode.MANUAL;
            }
        }
        // Raspberry Pi Targeted
        else if (driveMode == DriveMode.CARGO_TARGETED) {
            driveMode = DriveMode.MANUAL;
        }
    }

    /**
     * Controls the ball in TeleOp
     */
    private void ballControl() {
        // Controls 
        boolean deployRetract               = controls.grabberDeployRetract();
        boolean startShooter                = controls.startShooter();
        boolean secureBalls                 = controls.secureBalls();
        boolean releaseBalls                = controls.releaseBalls();
        Grabber.GrabberDirection grabberDir = controls.getGrabberDirection();
        Shooter.ShootLocation shootLocation = controls.getShootLocation();

        // Shooter Ready
        boolean isShooterReady              = shooter.shooterReady();

        /**
         * Grabber control
         */
        if (deployRetract == true) {
            grabber.deployRetract();
        }
        grabber.setGrabberMotor(grabberDir);

        /**
         * Ball blocker
         * <p> Manual controls takes precedent over automatic control
         * <p> Automatically brings the ball blocker up when grabberDeploy() is called
         */
        // Manual
        if ((secureBalls == true) && (releaseBalls == true)) {
            System.out.println("ERROR! BOTH BALL BLOCKER BUTTONS PRESSED!");
        }
        else if (secureBalls == true) {
            grabber.blockBalls();
        }
        else if (releaseBalls == true) {
            grabber.releaseBalls();
        }
        // Automatic
        else if (grabberDir == Grabber.GrabberDirection.INTAKE) {
            grabber.releaseBalls();
        }

        /**
         * Shooter control
         */
        if (shootLocation == Shooter.ShootLocation.OFF) {
            shooter.disableShooter();
            shooter.blockShooter();

            if (startShooter == true) {
                shooter.shooterControl(Shooter.ShootLocation.HIGH_SHOT);
            }
        }
        else {
            shooter.shooterControl(shootLocation);

            if (isShooterReady == true && (lowShotEnabled == true || driveMode == DriveMode.LIMELIGHT_TARGETED)) {
                shooter.openShooter();
                grabber.releaseBalls();
                led.shooterReady();
            }
            else if (isShooterReady == false) {
                shooter.blockShooter();
                led.shooterNotReady();
            }
            else {
                led.shooterNotReady();
            }

            led.updateShooter();
        }
    }

    /**
     * Controls the climber in TeleOp
     */
    private void climberControl() {
        // Claw toggles
        boolean toggleBlueClaw   = controls.toggleBlueClaw();
        boolean toggleYellowClaw = controls.toggleYellowClaw();

        // Movement functions
        double  climberPower = controls.getClimberPower();
        boolean moveToBar2   = controls.getClimberMoveToBar2();
        boolean moveToBar3   = controls.getClimberMoveToBar3();
        boolean moveToBar4   = controls.getClimberMoveToBar4();

        // Encoder reset
        boolean resetEncoder = controls.resetClimberEncoder();

        if (toggleBlueClaw == true) {
            climber.blueClawToggle();
        }
        if (toggleYellowClaw == true) {
            climber.yellowClawToggle();
        }

        if (moveToBar2 == true) {
            climber.moveToBar2();
        }
        else if (moveToBar3 == true) {
            climber.moveToBar3();
        }
        else if (moveToBar4 == true) {
            climber.moveToBar4();
        }
        else {
            climber.climberRotate(climberPower);
        }

        if (resetEncoder == true) {
            climber.resetEncoder();
        }
    }

    /**
     * Determines if we are on the red alliance
     * @return isRed
     */
    private boolean getRedAlliance() {
        return CustomTables.getRedAlliance();
    }
}

// End of the Robot class