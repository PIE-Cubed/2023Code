// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Start of the Grabber class
 */
public class Grabber extends SubsystemBase {
    // SPARK MAX ID's
    private static final int SPARK_ID = 18;

    // PNEUMATICS IDS
    private final int PCM_CAN_ID              = 1;
    private final int GRABBER_DEPLOY_ID       = 4;
    private final int GRABBER_RETRACT_ID      = 0;
    private final int BALL_BLOCKER_DEPLOY_ID  = 1;
	private final int BALL_BLOCKER_RETRACT_ID = 5;

    // SPARK MAX CURRENT LIMIT
    private int GRABBER_CURRENT_LIMIT = 60;

    // Spark Max motor
    private CANSparkMax grabberMotor;

    // Pistons
    private DoubleSolenoid ballBlocker;
    private DoubleSolenoid grabberPiston;

    // CONSTANTS
    private final double GRABBER_POWER = -0.75;

    /**
     * Enumerator for Grabber States
     */
    public static enum GrabberState {
        DEPLOY,
        RETRACT;
    }
    public static GrabberState grabberState;

    /**
     * Enumerater for Grabber Direction
     */
    public static enum GrabberDirection {
        INTAKE,
        EXPEL,
        OFF;
    }

    /**
     * The constructor for the Grabber class
     */
    public Grabber()  {
        // Grabber Motor Init
        grabberMotor = new CANSparkMax(SPARK_ID, MotorType.kBrushed);
        grabberMotor.setSmartCurrentLimit(GRABBER_CURRENT_LIMIT);
        grabberMotor.set(0.0);

        // Grabber Piston Init
        grabberPiston = new DoubleSolenoid(PCM_CAN_ID, PneumaticsModuleType.CTREPCM, GRABBER_DEPLOY_ID, GRABBER_RETRACT_ID);
        grabberPiston.set(Value.kReverse);
        grabberState = GrabberState.RETRACT;

        // Ball blocker Piston Init
        ballBlocker = new DoubleSolenoid(PCM_CAN_ID, PneumaticsModuleType.CTREPCM, BALL_BLOCKER_DEPLOY_ID, BALL_BLOCKER_RETRACT_ID);
        ballBlocker.set(Value.kReverse);
    }

    /*
     * EXTEND / RETRACT FORWARDING PISTON
     */
    public void deployRetract() {
        // Toggle the State of the Piston
        if (grabberState == GrabberState.DEPLOY) {
            grabberRetract();
        }
        else if (grabberState == GrabberState.RETRACT) {
            grabberDeploy();
        }
    }

    public void setGrabberMotor(GrabberDirection dir) {
        // Grabber Intake
        if (dir == GrabberDirection.INTAKE) {
            grabberMotor.set(GRABBER_POWER);
        }
        // Grabber Reverse
        else if (dir == GrabberDirection.EXPEL) {
            grabberMotor.set(GRABBER_POWER * -1);
        }
        // No direction
        else {
            grabberMotor.set(0.0);
        }
    }

    public void stopGrabber() {
        setGrabberMotor(GrabberDirection.OFF);
    }

    /**
     * Individual functions
     */
    /**
     * Deploys the grabber
     * <p> Automatically brings the ball blocker up
     */
    public void grabberDeploy() {
        grabberPiston.set(Value.kForward);
        grabberState = GrabberState.DEPLOY;
        releaseBalls();
    }

    public void grabberRetract() {
        grabberPiston.set(Value.kReverse);
        grabberState = GrabberState.RETRACT;
    }

    /**
	 * Deploys the ball blocker to stop balls from leaving the robot
	 */
	public void blockBalls() {
		ballBlocker.set(Value.kForward);
	}

	/**
	 * Retracts the ball blocker to allow balls to enter the robot
	 */
	public void releaseBalls() {
		ballBlocker.set(Value.kReverse);
	}
}

// End of the Grabber Class