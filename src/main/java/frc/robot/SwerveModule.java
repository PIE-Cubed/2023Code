// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Start of the SwerveModule class
 */
public class SwerveModule {
    // CONSTANTS
    private final int MOTOR_CURRENT_LIMIT = 80;

    // Constants for drive distance conversion
    private final double wheelDiameterInches      = 3.00;
    private final double ticksPerWheelRevolution  = 5.50;
    private final double inchesPerWheelRevolution = wheelDiameterInches * Math.PI;
    private final double inchesPerTick            = inchesPerWheelRevolution / ticksPerWheelRevolution;

    // Constants for module distance conversion
    private final double ticksPerModuleRevolution   = 10; //Unknown
    private final double radiansPermoduleRevolution = 2 * Math.PI;

    // Conversion Factors
    private final double metersDrivenPerTick   = Units.inchesToMeters(inchesPerTick);
    private final double radiansRotatedPerTick = radiansPermoduleRevolution / ticksPerModuleRevolution;

    // Create motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax rotateMotor;

    // Create encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotateEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    // Drive PID controller
    private final double DRIVE_P = 0.05;
    private final double DRIVE_I = 0.00;
    private final double DRIVE_D = 0.00;
    private PIDController driveMotorController;

    // Rotate PID Controller Variables
    private final double ROTATE_P = 0.05;
    private final double ROTATE_I = 0.00;
    private final double ROTATE_D = 0.00;
    private ProfiledPIDController rotateMotorController;

    // rotateMotorController limits
    private final double MAX_MODULE_ROTATE_SPEED        = 2 * Math.PI; // radians per second
    private final double MAX_MODULE_ROTATE_ACCELERATION = 1 * Math.PI; // radians per second per second
    private final Constraints ROTATE_CONSTRAINT = new Constraints(MAX_MODULE_ROTATE_SPEED, MAX_MODULE_ROTATE_ACCELERATION);

    // Integrator Range
    private static final double DRIVE_I_MAX  = 0.10;
    private static final double DRIVE_I_MIN  = -1 * DRIVE_I_MAX;
    private static final double ROTATE_I_MAX = 0.05;
    private static final double ROTATE_I_MIN = -1 * ROTATE_I_MAX;

    // Defines motor feedforward values
    private final double DRIVE_STATIC_GAIN    = 0;
    private final double DRIVE_VELOCITY_GAIN  = 0.33;
    private final double ROTATE_STATIC_GAIN   = 0;
    private final double ROTATE_VELOCITY_GAIN = 0.33;

    // Creates motor feedforward loops
    private final SimpleMotorFeedforward driveFeedforward;
    private final SimpleMotorFeedforward rotateFeedforward;

    /**
     * The constructor for the SwerveModule class
     *
     * @param driveID The CAN ID of the drive motor.
     * @param rotateMotorId The CAN ID of the rotate motor.
     * @param invertMotor Wheter the drive motor is reversed.
     */
    public SwerveModule(int driveID, int rotateID, boolean invertDriveMotor) {
        // Creates the drive motor
        driveMotor            = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(invertDriveMotor);

        // Creates the rotation motor
        rotateMotor           = new CANSparkMax(rotateID, MotorType.kBrushless);
        rotateMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        rotateMotor.setIdleMode(IdleMode.kBrake);

        // Creates the encoders
        driveEncoder          = driveMotor .getEncoder();
        rotateEncoder         = rotateMotor.getEncoder();
        absoluteEncoder       = rotateMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // Creates the rotate PID Controller
        rotateMotorController = new ProfiledPIDController(ROTATE_P, ROTATE_I, ROTATE_D, ROTATE_CONSTRAINT);
        rotateMotorController.enableContinuousInput(-Math.PI, Math.PI);

        // Creates the drive PID Controller
        driveMotorController  = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D);

        // Sets the motor conversion factors
        driveEncoder   .setPositionConversionFactor(metersDrivenPerTick);        // Converts to meters from revolutions
        driveEncoder   .setVelocityConversionFactor(metersDrivenPerTick / 60);   // Converts to meters/second from revolutions/minute
        rotateEncoder  .setPositionConversionFactor(radiansRotatedPerTick);      // Converts to radians from revolutions
        rotateEncoder  .setVelocityConversionFactor(radiansRotatedPerTick / 60); // Converts to radians/second from revolutions/minute
        absoluteEncoder.setPositionConversionFactor(-2 * Math.PI);               // Converts to radians from revolutions
        absoluteEncoder.setVelocityConversionFactor(-2 * Math.PI);               // Converts to radians/second from revolutions/second

        // Temporarily removes the rotateEncoder's conversion factors for testing
        rotateEncoder  .setPositionConversionFactor(1);
        rotateEncoder  .setVelocityConversionFactor(1 / 60); // Converts to revolutions/second from revolutions/minute

        // Inegrator Ranges
        driveMotorController.setIntegratorRange(DRIVE_I_MIN, DRIVE_I_MAX);
        rotateMotorController.setIntegratorRange(ROTATE_I_MIN, ROTATE_I_MAX);

        // Set the Rotate Controller's input to be -pi to pi
        rotateMotorController.enableContinuousInput(-Math.PI, Math.PI);

        // Creates the motors feedforward functions
        driveFeedforward  = new SimpleMotorFeedforward(DRIVE_STATIC_GAIN , DRIVE_VELOCITY_GAIN);
        rotateFeedforward = new SimpleMotorFeedforward(ROTATE_STATIC_GAIN, ROTATE_VELOCITY_GAIN);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d( getAbsoluteEncoderPosition() ));

        // Calculate the drive output from the drive PID controller.
        double driveOutput = driveMotorController.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        double driveFeedforwardPower = driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        double rotateOutput = rotateMotorController.calculate(getAbsoluteEncoderPosition(), state.angle.getRadians());
        double rotateFeedforwardPower = rotateFeedforward.calculate(rotateMotorController.getSetpoint().velocity);

        // Calculate the turning motor output from the turning PID controller.
        driveMotor .set(driveOutput  + driveFeedforwardPower);
        rotateMotor.set(rotateOutput + rotateFeedforwardPower);
    }

    /****************************************************************************************** 
    *
    *    GETTER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            new Rotation2d( getAbsoluteEncoderPosition() )
        );
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            new Rotation2d( getAbsoluteEncoderPosition() )
        );
    }

    /**
     * Gets the drive motor's position.
     * 
     * @return The drive motor's position in m
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Gets the rotate motor's position.
     * 
     * @return The rotate encoder's position in radians
     */
    public double getRotatePosition() {
        return absoluteEncoder.getPosition();
    }

    /**
     * Gets the drive motor's velocity.
     * 
     * @return The drive motor's velocity in m/s
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Gets the rotate motor's velocity.
     * 
     * @return The rotate motor's velocity in m/s
     */
    public double getRotateVelocity() {
        return absoluteEncoder.getVelocity();
    }

    /****************************************************************************************** 
    *
    *    HELPER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Changes the encoder's range from 0 to 2pi into -pi to pi.
     * 
     * @return The value converted into -pi to pi
     */
    private double getAbsoluteEncoderPosition() {
        return MathUtil.angleModulus(absoluteEncoder.getPosition());
    }

    /****************************************************************************************** 
    *
    *    TEST FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Inits the motor sliders on SmartDashboard.
     */
    public void initMotorSliders() {
        SmartDashboard.putNumber("Drive Motor Power", 0);
        SmartDashboard.putNumber("Rotate Motor Power", 0);
    }

    /**
     * Displays the encoder values on SmartDashboard.
     */
    public void displayEncoderValues() {
        SmartDashboard.putNumber(driveMotor.getDeviceId()  + "Drive Encoder", getDrivePosition());
        SmartDashboard.putNumber(rotateMotor.getDeviceId() + "Rotate Encoder", getRotatePosition());
        SmartDashboard.putNumber(rotateMotor.getDeviceId() + "Rotation Absolute Encoder", getAbsoluteEncoderPosition());
    }

    /**
     * Makes the motors drive.
     */
    public void updateMotorPowers() {
        setDriveMotorPower (SmartDashboard.getNumber("Drive Motor Power", 0));
        setRotateMotorPower(SmartDashboard.getNumber("Rotate Motor Power", 0));
    }

    /**
     * Zeros the position of the drive and rotate encoders.
     */
    public void zeroMotorEncoders() {
        driveEncoder .setPosition(0.00);
        rotateEncoder.setPosition(0.00);
    }

    /**
     * Drives a certain wheel at a certain power.
     * 
     * @param power
     */
    private void setDriveMotorPower(double power) {
        power = MathUtil.clamp(power, -1, 1);
        driveMotor.set(power);
    }

    /**
     * Rotates a certain wheel at a certain power.
     * 
     * @param power
     */
    private void setRotateMotorPower(double power) {
        power = MathUtil.clamp(power, -1, 1);    
        rotateMotor.set(power);
    }
}

// End of the SwerveModule class