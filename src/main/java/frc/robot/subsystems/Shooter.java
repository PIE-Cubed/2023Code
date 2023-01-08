// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Start of the Shooter class
 */
public class Shooter extends SubsystemBase {
	// 2 holes above marked one
	// Low shot against hub- 0.25 speed, 1380-1500 rpm
	// High shot from edge of tarmac- 0.5 speed, 3800-4000 rpm

	// 3 holes above marked one
	// Low against hub- 0.3 speed, 1650-1750 rpm
	// High from edge of tarmac- 0.525 speed, 2980-3100 rpm

	// SPARK MAX
	private CANSparkMax    leadShooter;   // Needs negative power to shoot
	private CANSparkMax    followShooter; // Needs positive power to shoot
	private DoubleSolenoid shooterBlocker;

	// SPARK MAX IDs
	private int LEAD_SHOOTER_ID   = 20; // Lead is right
	private int FOLLOW_SHOOTER_ID = 19; // Follow is left

	// Pneumatics IDs
	private final int SHOOTER_BLOCKER_DEPLOY_ID  = 7;
	private final int SHOOTER_BLOCKER_RETRACT_ID = 0;
	private final int PCM_CAN_ID                 = 2;

	// Encoders
	private RelativeEncoder shooterEncoder;

	// POWER CONSTANTS
	// Find power required to get to target rpm w/o PID Subtract by 0.03
	// Use PID with mostly I, the P will just give a boost at the start
	public final double OFF_POWER        =  -0.00;
	public final double LOW_SHOT_POWER   =  -0.35; 
	public final double HIGH_SHOT_POWER  =  -0.63; 
	public final double LAUNCH_PAD_POWER =  -0.75; 
	public final double AUTO_RING_POWER  =  -0.64;

	// RPM CONSTANTS
	public final double OFF_TARGET_RPM         = 6000; // Don't want shooter ready to be true while off

	//Against hub
	public final double LOW_SHOT_TARGET_RPM    = 2100; 

	// 9 feet and 6 inches from the center of the hub
	public final double HIGH_SHOT_TARGET_RPM   = 3750; 

	// 16 feet and 10 inches from the center of the hub
	public final double LAUNCH_PAD_TARGET_RPM  = 4250; 

	// 12 feet and 8 inches from the center of the hub
	public final double AUTO_RING_TARGET_RPM   = 3850; 

	// RPM OFFSET
	private final int RPM_OFFSET = 50;

	// Current limit constant
	private static final int SHOOTER_CURRENT_LIMIT = 80;

	// Variables
	public  double                targetVelocity = 0;
	private int                   targetCount    = 0;
	private double                power          = 0;

	// Enums
	public static enum ShootLocation {
		HIGH_SHOT,
		LOW_SHOT,
		LAUNCH_PAD,
		AUTO_RING,
		OFF;
	}

	public static enum BallFeederDirection {
		FORWARD,
		REVERSE,
		OFF;
	}

	// Shooter PID Controller
	private PIDController shooterController;

	// Integrator Constants
	private static final double MIN_INTEGRATOR = -0.1; 
	private static final double MAX_INTEGRATOR =  0.1; 

	private static final double kP = 0.00008; 
	private static final double kI = 0.0006; 
 	private static final double kD = 0.00;

	/**
	 * The constructor for the Shooter class
	 */
	public Shooter() {
		// SPARK Max
		leadShooter    = new CANSparkMax(LEAD_SHOOTER_ID, MotorType.kBrushless); // Shooter 2 requires positive power to shoot
		followShooter  = new CANSparkMax(FOLLOW_SHOOTER_ID, MotorType.kBrushless); // Shooter 1 requires negative power to shoot
		shooterBlocker = new DoubleSolenoid(PCM_CAN_ID, PneumaticsModuleType.CTREPCM, SHOOTER_BLOCKER_DEPLOY_ID, SHOOTER_BLOCKER_RETRACT_ID);

		// Sets the current limtis for the motors
		leadShooter  .setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);
		followShooter.setSmartCurrentLimit(SHOOTER_CURRENT_LIMIT);

		// Sets the mode of the motors
		leadShooter  .setIdleMode(CANSparkMax.IdleMode.kCoast);
		followShooter.setIdleMode(CANSparkMax.IdleMode.kCoast);

		// Reverses the the floowing motor
		followShooter.follow(leadShooter, true);

		// Set Shooter related motors to off to Start the Match
		leadShooter  .set(0.00);
		followShooter.set(0.00);

		// Resets the piston
		blockShooter();

		// Encoders
		shooterEncoder = leadShooter.getEncoder();

		// PID Controller
		shooterController = new PIDController(kP, kI, kD);
		shooterController.setIntegratorRange(MIN_INTEGRATOR, MAX_INTEGRATOR);
	}

	// Controls RPM of shooter
	public void shooterControl(ShootLocation location) {
		double powerError;
		
		if (location == ShootLocation.HIGH_SHOT) {
			powerError     = shooterController.calculate( getabsRPM(), HIGH_SHOT_TARGET_RPM);
			targetVelocity = HIGH_SHOT_TARGET_RPM;
			power          = HIGH_SHOT_POWER;
		}
		else if (location == ShootLocation.LOW_SHOT) {
			powerError     = shooterController.calculate( getabsRPM(), LOW_SHOT_TARGET_RPM);
			targetVelocity = LOW_SHOT_TARGET_RPM;
			power          = LOW_SHOT_POWER;
		}
		else if (location == ShootLocation.LAUNCH_PAD) {
			powerError     = shooterController.calculate( getabsRPM(), LAUNCH_PAD_TARGET_RPM);
			targetVelocity = LAUNCH_PAD_TARGET_RPM;
			power          = LAUNCH_PAD_POWER;
		}
		else if (location == ShootLocation.AUTO_RING) {
			powerError     = shooterController.calculate( getabsRPM(), AUTO_RING_TARGET_RPM);
			targetVelocity = AUTO_RING_TARGET_RPM;
			power          = AUTO_RING_POWER;
		}
		else {
			powerError      = OFF_POWER;
			targetVelocity  = OFF_TARGET_RPM;
			power           = OFF_POWER;
		}

		power = power - powerError;
		power = MathUtil.clamp(power, -1.00, 1.00);

		leadShooter.set(power);
	}

	/****************************************************************************************** 
    *
    *    shooterReady()
	*    Checks if shooter is ready to fire
    *   
    ******************************************************************************************/
	public boolean shooterReady() {
		// Variables
		double rpm;
		int ON_TARGET_DELAY = 5;

		// Gets rpm values
		rpm  = getabsRPM();

		// Calculates tolerable RPM range
		double lowerLimit  = targetVelocity  - RPM_OFFSET;
		double upperLimit  = targetVelocity  + RPM_OFFSET;

		if ((targetVelocity > 1) && (rpm > lowerLimit) && (rpm < upperLimit)) {
			targetCount ++;
			return (targetCount >= ON_TARGET_DELAY);
		}
		else {
			// Resets targetCount
			targetCount = 0;
			return false;
		}
	}
	
	/**
	 * Deploys the shooter blocker to stop balls
	 */
	public void blockShooter() {
		shooterBlocker.set(Value.kForward);
	}

	/**
	 * Retracts the shooter blocker to release balls
	 */
	public void openShooter() {
		shooterBlocker.set(Value.kReverse);
	}

	/**
	 * DEBUG / TEST FUNCTIONS
	 */
	/**
	 * Debug function to disable all motors that are controlled by this class
	 */
	public void disableShooter(){
		leadShooter.set(0.00);
		blockShooter();
	}

	/**
	 * Gets the abs RPM of the passed motor
	 * @return absRPM
	 */
	public double getabsRPM() {
		double absRPM = Math.abs(shooterEncoder.getVelocity());
		return absRPM;
	}

	/**
	 * 
	 * @param power
	 */
	public void testShootMotor(double power) {
		// Lead shooter motor needs to be negative to shoot a ball
		// Follow shooter motor (rear motor) needs to be positive to shoot a ball
		leadShooter.set(power);
		// SmartDashboard.putNumber("RPM", getabsRPM());
	}
}

// End of the Shooter Class