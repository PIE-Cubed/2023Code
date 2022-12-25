// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

/**
 * Start of the SwerveModule class
 */
public class SwerveModule {
    // CONSTANTS
    private final int WHEEL_CURRENT_LIMIT = 80;

    // Wheel Measurements
    private final double wheelDiameterInches = 4.00;
    private final double wheelDiameterMeters = Units.inchesToMeters(wheelDiameterInches);

    // Encoder Values
    private final double ticksPerFoot        = 5.65;
    private final double ticksPerRevolution  = ticksPerFoot / wheelDiameterInches / 12;

    // Distance Calculations
    private final double inchesDrivenPerTick  = (wheelDiameterInches * Math.PI) / ticksPerRevolution;
    private final double metersDrivenPerTick  = (wheelDiameterMeters * Math.PI) / ticksPerRevolution;
    private final double degreesRotatedPerTick = 360 / ticksPerRevolution;
    private final double radiansRotatedPerTick = (2 * Math.PI) / ticksPerRevolution;

    // Create motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax rotateMotor;

    // Create encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotateEncoder;

    // Create potentiometer
    private final AnalogPotentiometer absoluteEncoder;

    // Create a drive speed PID
    private final double driveP = 0.045;
    private final double driveI = 0.001;
    private final double driveD = 0.000;
    private PIDController driveController = new PIDController(driveP, driveI, driveD);

    // TrapezoidProfile PIDController variables
    private final double rotateP = 0.030;
    private final double rotateI = 0.001;
    private final double rotateD = 0.000;

    // TrapezoidProfile PIDController limits
    public final double MAX_MODULE_ROTATE_SPEED        = 2 * Math.PI;
    public final double MAX_MODULE_ROTATE_ACCELERATION = 2 * Math.PI;

    // Create a TrapezoidProfile PIDController for smooth turning
    private ProfiledPIDController rotateController =
        new ProfiledPIDController(
            rotateP, rotateI, rotateD,
            new TrapezoidProfile.Constraints(MAX_MODULE_ROTATE_SPEED, MAX_MODULE_ROTATE_ACCELERATION));

    // Integrator Range
    private static final double DRIVE_I_MAX  = 0.10;
    private static final double DRIVE_I_MIN  = -1 * DRIVE_I_MAX;
    private static final double ROTATE_I_MAX = 0.05;
    private static final double ROTATE_I_MIN = -1 * ROTATE_I_MAX;

    /**
     * The constructor for the SwerveModule class
     *
     * @param driveMotorId The channel of the drive motor.
     * @param rotateMotorId The channel of the turning motor.
     * @param absoluteSensorId The channel of the absolute encoder. 
     * @param driveMotorInverted Whether the drive motor is reversed.
     * @param rotateMotorInverted Wheter the rotate motor is reversed.
     * @param offsetDegrees The absolute encoder offset in degrees.
     */
    public SwerveModule(int driveMotorId, int rotateMotorId, int absoluteSensorId, boolean driveMotorInverted, boolean rotateMotorInverted, double offsetDegrees) {
        // Creates the drive motor and encoder
        driveMotor   = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();

        // Creates the rotate motor and encoder
        rotateMotor   = new CANSparkMax(rotateMotorId, MotorType.kBrushless);
        rotateEncoder = rotateMotor.getEncoder();

        // Creates an absolute encoder
        absoluteEncoder = new AnalogPotentiometer(absoluteSensorId, -360, offsetDegrees);

        // Adds current limits
        driveMotor.setSmartCurrentLimit(WHEEL_CURRENT_LIMIT);
        rotateMotor.setSmartCurrentLimit(WHEEL_CURRENT_LIMIT);

        // Sets the motor modes
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);;
        rotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Invert the motors if necessary
        driveMotor.setInverted(driveMotorInverted);
        rotateMotor.setInverted(rotateMotorInverted);

        // Sets the motor conversion factors
        driveEncoder.setPositionConversionFactor(metersDrivenPerTick);
        driveEncoder.setVelocityConversionFactor(metersDrivenPerTick);
        rotateEncoder.setPositionConversionFactor(radiansRotatedPerTick);
        rotateEncoder.setVelocityConversionFactor(radiansRotatedPerTick);

        // Sets the velocity measurement periods
        driveEncoder.setMeasurementPeriod(100);
        rotateEncoder.setMeasurementPeriod(100);

        // Inegrator Ranges
        driveController.setIntegratorRange(DRIVE_I_MIN, DRIVE_I_MAX);
        rotateController.setIntegratorRange(ROTATE_I_MIN, ROTATE_I_MAX);

        // Limit the PID Controller's input range between -pi and pi and set the input to be continuous.
        rotateController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * getState()
     * <p>Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d( degToRad(getRotationPosition()) ));
    }

    /**
     * setDesiredState()
     * <p>Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d( degToRad(getRotationPosition()) ));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = driveController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = rotateController.calculate(getRotationPosition(), state.angle.getDegrees());

        // Calculate the turning motor output from the turning PID controller.
        driveMotor.set(driveOutput);
        rotateMotor.set(turnOutput);
    }

    /**
     * degtoRad()
     * <p>Converts degrees to radians.
     * 
     * @param degrees
     * @return radians
     */
    private double degToRad(double degrees) {
        return Units.degreesToRadians(degrees);
    }

    /**
     * getRotationPosition()
     * <p>Returns rotate encoder values in a range from -180 to 180
     * 
     * @return absoluteEncoderPosition
     */
    public double getRotationPosition() {
        double rawAngle = absoluteEncoder.get();
        return normalizeAngle(rawAngle);
    }

    /**
     * normalizeAngle()
     * <p>Takes an input of degrees and brings it from (0 to 360) to (-180 to 180)
     * 
     * @param degrees
     * @return adjustedValue
     */
    private double normalizeAngle(double degrees){
        double adjustedValue = degrees;

        if ((adjustedValue >= -180) && (adjustedValue <= 180)) {
            // Does nothing to adjustedValue
        }
        else if (adjustedValue > 180) {
            // Makes all values greater than 180 less than it
            adjustedValue -= 360;
        }
        else if (adjustedValue < -180) {
            // Makes all values less than -180 greater than it
            adjustedValue += 360;
        }

        // This should only run if degrees is greater than 360 or less than -360
        if (adjustedValue < -180 || adjustedValue > 180) {
            adjustedValue = normalizeAngle(adjustedValue);
        }

        return adjustedValue;
    }

    /**
     * getEncoderValue()
     * <p>Gets the drive motor encoder value
     * 
     * @return driveMotorPosition
     */
    public double getEncoderValue() {
        return driveEncoder.getPosition();
    }

    /**
     * 
     * TEST METHODS
     * 
     */

    /**
     * Drives a certain wheel at a certain power
     * @param power
     */
    public void setDriveMotorPower(double power) {
        power = MathUtil.clamp(power, -1, 1);
        driveMotor.set(power);
    }

    /**
     * Rotates a certain wheel at a certain power
     * @param power
     */
    public void setRotateMotorPower(double power) {
        power = MathUtil.clamp(power, -1, 1);    
        rotateMotor.set(power);
    }

    /**
     * testWheelAngle()
     * <p>Returns the direct reading of the rotate motor sensor
     * 
     * @return rotatePosition
     */
    public double testWheelAngle() {
        return getRotationPosition();
    }
}

// End of the SwerveModule class