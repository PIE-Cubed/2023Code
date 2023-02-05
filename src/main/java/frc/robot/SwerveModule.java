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

    // Drive Motor Conversion Factors
    private final double WHEEL_DIAMETER_METERS       = Units.inchesToMeters(3);
    private final double METERS_PER_ROTATION         = Math.PI * WHEEL_DIAMETER_METERS;
    private final double ROTATIONS_PER_TICK          = 1 / 5.5;
    private final double DRIVE_POS_CONVERSION_FACTOR = METERS_PER_ROTATION * ROTATIONS_PER_TICK; // Meters per tick
    private final double DRIVE_VEL_CONVERSION_FACTOR = DRIVE_POS_CONVERSION_FACTOR / 60; // m/s per tick/min (tick/min --> m/s)

    // Absolute Encoder Conversion Factors
    private final double MODULE_POS_CONVERSION_FACTOR = -1 * 2 * Math.PI;             // Meters per tick (tick --> meter)
    private final double MODULE_VEL_CONVERSION_FACTOR = MODULE_POS_CONVERSION_FACTOR; // m/s per tick/min (tick/min --> m/s)

    // Create motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax rotateMotor;

    // Create encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotateEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    // Drive PID controller
    private final double DRIVE_P = 0.01;
    private final double DRIVE_I = 0.01;
    private final double DRIVE_D = 0.00;
    private PIDController driveMotorController;

    // Rotate PID Controller Variables
    private final double ROTATE_P = 0.20;
    private final double ROTATE_I = 0.01;
    private final double ROTATE_D = 0.00;
    private ProfiledPIDController rotateMotorController;

    // rotateMotorController limits
    private final double MAX_MODULE_ROTATE_SPEED        = 2 * Math.PI; // radians per second
    private final double MAX_MODULE_ROTATE_ACCELERATION = 1 * Math.PI; // radians per second per second
    private final Constraints ROTATE_CONSTRAINT = new Constraints(MAX_MODULE_ROTATE_SPEED, MAX_MODULE_ROTATE_ACCELERATION);

    // Integrator Range
    private static final double DRIVE_I_MAX  = 0.10;
    private static final double DRIVE_I_MIN  = -1 * DRIVE_I_MAX;
    private static final double ROTATE_I_MAX = 0.10;
    private static final double ROTATE_I_MIN = -1 * ROTATE_I_MAX;

    // Defines motor feedforward values
    private final double DRIVE_STATIC_GAIN    = 0;
    private final double DRIVE_VELOCITY_GAIN  = 0.12;

    // Creates motor feedforward loops
    private final SimpleMotorFeedforward driveFeedforward;

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

        // Resets the encoders
        driveEncoder .setPosition(0.00);
        rotateEncoder.setPosition(0.00);

        // Creates the drive PID Controller
        driveMotorController  = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D);

        // Creates the rotate PID Controller
        rotateMotorController = new ProfiledPIDController(ROTATE_P, ROTATE_I, ROTATE_D, ROTATE_CONSTRAINT);
        rotateMotorController.enableContinuousInput(-Math.PI, Math.PI);

        // Sets the motor conversion factors
        driveEncoder   .setPositionConversionFactor(DRIVE_POS_CONVERSION_FACTOR);   // Converts from revolutions to meters
        driveEncoder   .setVelocityConversionFactor(DRIVE_VEL_CONVERSION_FACTOR);   // Converts from revolutions/minute to meters/second
        absoluteEncoder.setPositionConversionFactor(MODULE_POS_CONVERSION_FACTOR);  // Converts from revolutions to radians
        absoluteEncoder.setVelocityConversionFactor(MODULE_VEL_CONVERSION_FACTOR);  // Converts from revolutions/second to radians/second

        // Integrator Ranges
        driveMotorController .setIntegratorRange(DRIVE_I_MIN , DRIVE_I_MAX);
        rotateMotorController.setIntegratorRange(ROTATE_I_MIN, ROTATE_I_MAX);

        // Set the Rotate Controller's input to be -pi to pi
        rotateMotorController.enableContinuousInput(-Math.PI, Math.PI);

        // Creates the motor feedforward
        driveFeedforward = new SimpleMotorFeedforward(DRIVE_STATIC_GAIN , DRIVE_VELOCITY_GAIN);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimizes the wheel movements
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d( getAbsPosition() ));

        // Calculates the rotate power
        double currentAngle = getAbsPosition();
        double targetAngle  = MathUtil.angleModulus( optimizedState.angle.getRadians() );
        double rotatePower  = rotateMotorController.calculate(currentAngle, targetAngle);

        // Calculates the drive power
        double currentSpeed = getDriveVelocity();
        double targetSpeed  = optimizedState.speedMetersPerSecond;
        double feedForward  = driveFeedforward    .calculate(targetSpeed);
        double pidError     = driveMotorController.calculate(currentSpeed, targetSpeed);

        // Sets motor powers
        driveMotor .set(feedForward + pidError);
        rotateMotor.set(rotatePower);
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
            getDrivePosition(),
            new Rotation2d( getAbsPosition() )
        );
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            new Rotation2d( getAbsPosition() )
        );
    }

    /**
     * Gets the drive motor's position.
     * 
     * @return The drive motor's position in meters
     */
    private double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Gets the rotate motor's position.
     * 
     * @return The rotate encoder's position in ticks
     */
    private double getRotatePosition() {
        return rotateEncoder.getPosition();
    }

    /**
     * Gets the absolute encoder's position.
     * 
     * @return The absolute encoder's position in radians
     */
    private double getAbsPosition() {
        return MathUtil.angleModulus( absoluteEncoder.getPosition() );
    }

    /**
     * Gets the drive motor's velocity.
     * 
     * @return The drive motor's velocity in meters/second
     */
    private double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    // /**
    //  * Gets the rotate motor's velocity.
    //  * 
    //  * @return The rotate motor's velocity in ticks/second
    //  */
    // private double getRotateVelocity() {
    //     return rotateEncoder.getVelocity();
    // }

    // /**
    //  * Gets the absolute encoder's velocity.
    //  * 
    //  * @return The absolute encoder's velocity in radians/second
    //  */
    // private double getAbsVelocity() {
    //     return absoluteEncoder.getVelocity();
    // }

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
        SmartDashboard.putNumber(rotateMotor.getDeviceId() + "Rotation Absolute Encoder", getAbsPosition());
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
        //rotateEncoder.setPosition(0.00);
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