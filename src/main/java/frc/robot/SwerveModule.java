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

    // Wheel Measurements
    private final double wheelDiameterInches = 3.00;
    private final double wheelDiameterMeters = Units.inchesToMeters(wheelDiameterInches);

    // Drive and Rotate Motor Constants
    private final double ticksPerWheelRevolution  = 5.50;
    private final double ticksPerModuleRevolution = 5.50;

    // Distance Conversion Factors
    private final double metersDrivenPerTick   = (wheelDiameterMeters * Math.PI) / ticksPerWheelRevolution;
    private final double radiansRotatedPerTick = (2 * Math.PI) / ticksPerModuleRevolution;

    // Create motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax rotateMotor;

    // Create encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotateEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    // Drive PID controller
    private final double DRIVE_P = 0.00;
    private final double DRIVE_I = 0.00;
    private final double DRIVE_D = 0.00;
    private PIDController driveMotorController;

    // Rotate PID Controller Variables
    private final double ROTATE_P = 0.20;
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

    // Creates motor feedforward loops (These must be tuned)
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3.0);
    private final SimpleMotorFeedforward m_turnFeedforward  = new SimpleMotorFeedforward(1, 0.5);

    /**
     * The constructor for the SwerveModule class
     *
     * @param driveID The CAN ID of the drive motor.
     * @param rotateMotorId The CAN ID of the turning motor.
     * @param invertMotor Wheter the rotate motor is reversed.
     */
    public SwerveModule(int driveID, int rotateID, boolean invertMotor) {
        // Creates the drive motor
        driveMotor            = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(invertMotor);

        // Creates the rotation motor
        rotateMotor           = new CANSparkMax(rotateID, MotorType.kBrushless);
        rotateMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        rotateMotor.setIdleMode(IdleMode.kBrake);

        // Creates the encoders
        driveEncoder          = driveMotor.getEncoder();
        rotateEncoder         = rotateMotor.getEncoder();
        absoluteEncoder       = rotateMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // Creates the rotate PID Controller
        rotateMotorController = new ProfiledPIDController(ROTATE_P, ROTATE_I, ROTATE_D, ROTATE_CONSTRAINT);
        rotateMotorController.enableContinuousInput(-Math.PI, Math.PI);

        // Creates the drive PID Controller
        driveMotorController = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D);

        // Sets the motor conversion factors
        driveEncoder.setPositionConversionFactor(metersDrivenPerTick);
        driveEncoder.setVelocityConversionFactor(metersDrivenPerTick);
        rotateEncoder.setPositionConversionFactor(radiansRotatedPerTick);
        rotateEncoder.setVelocityConversionFactor(radiansRotatedPerTick);
        absoluteEncoder.setVelocityConversionFactor(-2 * Math.PI);
        absoluteEncoder.setVelocityConversionFactor(-2 * Math.PI);

        // Inegrator Ranges
        driveMotorController.setIntegratorRange(DRIVE_I_MIN, DRIVE_I_MAX);
        rotateMotorController.setIntegratorRange(ROTATE_I_MIN, ROTATE_I_MAX);

        // Set the Rotate Controller's input to be -pi to pi
        rotateMotorController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void move(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d( getAdjustedAbsoluteEncoder() ));

        // Calculate the drive output from the drive PID controller.
        double driveOutput = driveMotorController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);
        double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        double rotateOutput = rotateMotorController.calculate(getAdjustedAbsoluteEncoder(), state.angle.getDegrees());
        double rotateFeedforward = m_turnFeedforward.calculate(rotateMotorController.getSetpoint().velocity);

        // Removes the value from the feedForward functions
        driveFeedforward  = 0;
        rotateFeedforward = 0;

        // Calculate the turning motor output from the turning PID controller.
        driveMotor .set(driveOutput  + driveFeedforward);
        rotateMotor.set(rotateOutput + rotateFeedforward);
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
            new Rotation2d( getAdjustedAbsoluteEncoder() )
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
            new Rotation2d( getAdjustedAbsoluteEncoder() )
        );
    }

    /**
     * 
     * 
     * @return
     */
    private double getDriveEncoder() {
        return driveEncoder.getPosition();
    }

    /**
     * 
     * 
     * @return
     */
    private double getRotateEncoder() {
        return rotateEncoder.getPosition();
    }

    /****************************************************************************************** 
    *
    *    HELPER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Changes the encoder's range to -pi to pi
     * 
     * @return adjustedRadian
     */
    private double getAdjustedAbsoluteEncoder() {
        double rad = -2 * Math.PI * absoluteEncoder.getPosition();

        rad = MathUtil.angleModulus(rad);

        return rad;
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
        SmartDashboard.putNumber(driveMotor.getDeviceId()  + "Drive Encoder", getDriveEncoder());
        SmartDashboard.putNumber(rotateMotor.getDeviceId() + "Rotate Encoder", getRotateEncoder());
        SmartDashboard.putNumber(rotateMotor.getDeviceId() + "Rotation Absolute Encoder", getAdjustedAbsoluteEncoder());
    }

    /**
     * Makes the motors drive.
     */
    public void updateMotorPowers() {
        setDriveMotorPower (SmartDashboard.getNumber("Drive Motor Power", 0));
        setRotateMotorPower(SmartDashboard.getNumber("Rotate Motor Power", 0));
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