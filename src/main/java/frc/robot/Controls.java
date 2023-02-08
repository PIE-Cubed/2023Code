// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Start of the Controls class
 */
public class Controls {
	// CONSTANTS
	private final int JOYSTICK_ID = 1;
	private final int XBOX_ID     = 0;

	// Controller object declaration
	private Joystick       joystick;
	private XboxController xboxController;

	// Rate limiters
	private SlewRateLimiter xLimiter;
	private SlewRateLimiter yLimiter;
	private SlewRateLimiter rotateLimiter;

	//Constructor
	public Controls() {
		// Instance Creation
		joystick       = new Joystick(JOYSTICK_ID);
		xboxController = new XboxController(XBOX_ID);

		// Create the rate limiters
		xLimiter      = new SlewRateLimiter(6); // -6 to 6 in two seconds
		yLimiter      = new SlewRateLimiter(6); // -6 to 6 in two seconds
		rotateLimiter = new SlewRateLimiter(6 * Math.PI); 
	}


	/* TJM
	 * In the past we had to add deadband because the robot would drive off on us 
	 * without touching the joystick.  I haven't seen that on this robot.  In addition
	 * using a deadband removes some fine control at low speeds.
	 * 
	 * It appears this code is always returning the max speed limited by the slew rate
	 * 
	 * I'm not so sure we want to use a slewRateLimiter here.  As long as we give the driver
	 * fine control at low speeds we can immediately give the driver the power they ask for.
	 * Not sure what prompted adding slewRateLimiter however...
	 * If it was put here to prevent skidding and wheels breaking loose to help encoder based 
	 * odometry from collecting errors I think we would be fighting a loosing battle there.
	 * A few bumps from another robot on our small robot and I think the encoder based odometry
	 * is shot.
	 * 
	 * I think we can quickly test cubing/quad etc the controls to get the control we need at
	 * low speeds.
	 * 
	 * I think we have 2 choices here.  We can translate the powers into a reasonable speed in m/sec or
	 * we could just use the -1 to +1 values that we cube to represent speed in m/sec.
	 * Translation to speeds will require a little collection of data off the robot.
	 * Using real speeds allow us to use a pid for velocity on each wheel.  The hope is that
	 * this will help the robot drive straight.  Not proven yet.  But this is really a autonomous issue and
	 * doesn't affect teleop drive too much.
	 * If we just use the the joystick values as a representation of speed we could skip the pid for velocity
	 * and just feed the values into the motor power when desaturated to +-1.
	 * I'm not sure how well the velocity pid on the forward power will work when comparing joystick power
	 * to actual velocity off the wheel encoder.
	 * 
	 * I don't think we ever use these values by themselves.  We could just return a ChassisSpeeds object
	 * here if you'd like.  No real gain here either way. Just a little more compact.
	 */
	/**
	 * DRIVE FUNCTIONS
	 */
	/**
	 * Gets the forward speed
	 * <p>Forward is positive to match chassis speed standards
	 * <p>This measures rotatation around the Y axis, which is effectively translation on the X axis
	 * 
	 * @return forwardSpeed
	 */
	public double getForwardSpeed() {
		/* The joystick produces a negative value when pushed forward
		 * negate the value to produce positive values when joystick pushed forward
		 */
		double speed = -1 * joystick.getY();

		// If we are in deadzone, y is 0
		speed = MathUtil.applyDeadband(speed, 0.1, 1);

		//
		speed *= Drive.getMaxSpeed();

		// 
		speed = xLimiter.calculate(speed);

		return speed;
	}

	/**
	 * Gets the strafe speed
	 * <p>Left is positive to match chassis speed standards
	 * <p>This measures rotatation around the X axis, which is effectively translation on the Y axis
	 * 
	 * @return strafeSpeed
	 */
	public double getStrafeSpeed() {
		/* The joystick produces a negative value when pushed left
		 * negate the value to produce positive values when joystick pushed left
		 */
		double speed = -1 * joystick.getX();

		// If we are in deadzone, x is 0
		speed = MathUtil.applyDeadband(speed, 0.1, 1);

		//
		speed *= Drive.getMaxSpeed();

		//
		speed = yLimiter.calculate(speed);

		return speed;
	}

	/**
	 * Gets the rotate speed
	 * <p>Counterclockwise is positive to match chassis speed standards
	 * <p>This measures rotatation around the Z axis
	 * 
	 * @return rotateSpeed
	 */
	public double getRotateSpeed() {
		double speed = -1 * joystick.getZ(); 

		// If we are in deadzone, rotatespeed is 0
		speed = MathUtil.applyDeadband(speed, 0.15, 1);

		//
		speed *= Drive.getMaxRotationSpeed();

		//
		speed = rotateLimiter.calculate(speed);

		return speed;    
	}


	/**
	 * ARM CONTROLS
	 */



	/**
	 * WRIST CONTROLS
	 */



	/**
	 * CLAW CONTROLS
	 */



	/**
	 * Checks if the start button is pressed
	 * @return start button pressed
	 */
	public boolean autoKill() {
		return xboxController.getStartButtonPressed();
	}
}

// End of the Controls class