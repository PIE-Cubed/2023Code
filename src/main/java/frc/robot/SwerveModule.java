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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Start of the SwerveModule class
 */
public class SwerveModule {
    // Object Creation
    private CANSparkMax            driveMotor;
    private CANSparkMax            rotateMotor;
    private RelativeEncoder        driveEncoder;
    private AbsoluteEncoder        absoluteEncoder;
    private PIDController          driveMotorController;
    private PIDController          rotateMotorController;
    private SimpleMotorFeedforward driveFeedForward;

    // Variables
    private double drivePIDPrevious = 0;
    private double prevPower        = 0;

    // Constants
    private final int    MOTOR_CURRENT_LIMIT   = 80;
    
    // Controller Parameters
    private final double DRIVE_P               = 0.01;
    private final double DRIVE_I               = 0;
    private final double DRIVE_D               = 0;

    private final double ROTATE_P              = 0.2;
    private final double ROTATE_I              = 0;
    private final double ROTATE_D              = 0;

    private final double STATIC_GAIN           = 0;
    private final double VELOCITY_GAIN         = 0.12;

    // Conversion Factors
    private final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);
    private final double WHEEL_ROTATION_METERS = Math.PI * WHEEL_DIAMETER_METERS;
    private final double ROTATIONS_PER_TICK    = 1 / 5.5;
    private final double POS_CONVERSION_FACTOR = WHEEL_ROTATION_METERS * ROTATIONS_PER_TICK; // Meters per tick (tick --> meter). 1.05 is magic number
    private final double VEL_CONVERSION_FACTOR = POS_CONVERSION_FACTOR / 60; // m/s per tick/min (tick/min --> m/s)

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
        absoluteEncoder       = rotateMotor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(1);
        absoluteEncoder.setInverted(false);
        driveEncoder.setPosition(0);

        // Creates the rotate PID Controller
        rotateMotorController = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D);
        rotateMotorController.enableContinuousInput(-Math.PI, Math.PI);

        // Wide tolerance to not delay auto
        rotateMotorController.setTolerance(0.15); 

        driveMotorController  = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D);
        driveMotorController.enableContinuousInput(-Math.PI, Math.PI);

        driveFeedForward      = new SimpleMotorFeedforward(STATIC_GAIN, VELOCITY_GAIN);

        driveEncoder.setPositionConversionFactor(POS_CONVERSION_FACTOR);
        driveEncoder.setVelocityConversionFactor(VEL_CONVERSION_FACTOR);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimizes the wheel movements
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d( getAdjustedAbsoluteEncoder() ));

        // Rotate motor
        double currentAngle = getAdjustedAbsoluteEncoder();
        double targetAngle  = optimizedState.angle.getRadians();
        targetAngle = MathUtil.angleModulus(targetAngle);
        rotateMotorController.setSetpoint(targetAngle);

        double rotatePower  = rotateMotorController.calculate(currentAngle, targetAngle);
        rotateMotor.set(rotatePower);

        // Drive motor
        double currentSpeed = driveEncoder.getVelocity();
        double targetSpeed  = optimizedState.speedMetersPerSecond;

        double feedForward  = driveFeedForward.calculate(targetSpeed);
        double pidError     = driveMotorController.calculate(currentSpeed, targetSpeed);

        double drivePower   = feedForward + pidError + drivePIDPrevious;
        drivePIDPrevious    = pidError;

        prevPower = feedForward + pidError;
        driveMotor.set(drivePower);
    }

    public void directMove(SwerveModuleState desiredState) {
        // Rotate motor
        double currentAngle = getAdjustedAbsoluteEncoder();
        double targetAngle  = desiredState.angle.getRadians();
        targetAngle = MathUtil.angleModulus(targetAngle);

        double rotatePower  = rotateMotorController.calculate(currentAngle, targetAngle);
        rotateMotor.set(rotatePower);

        // Drive motor
        driveMotor.set(desiredState.speedMetersPerSecond);
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
            getDriveVelocity(),
            new Rotation2d( getAdjustedAbsoluteEncoder() )
        );
    }

    /**
     * Gets the drive motor's position.
     * 
     * @return The drive motor's position in meters
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Gets the drive motor's velocity.
     * 
     * @return The drive motor's velocity in meters/second
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public boolean rotateControllerAtSetpoint() {
        return rotateMotorController.atSetpoint();
    }

    /**
     * Gets the absolute encoder's position.
     * 
     * @return The absolute encoder's position in radians
     */
    public double getAdjustedAbsoluteEncoder() {
        double rad = -2 * Math.PI * absoluteEncoder.getPosition();
        rad = MathUtil.angleModulus(rad);

        return rad;
    }

    /****************************************************************************************** 
    *
    *    setRotateMotorPower()
    *    Rotates a certain wheel at a certain power
    * 
    ******************************************************************************************/
    public void setRotateMotorPower(double power) {
        power = MathUtil.clamp(power, -1, 1);    
        rotateMotor.set(power);
    }

    /****************************************************************************************** 
    *
    *    setDriveMotorPower()
    *    Powers individual drive motors
    *    Left side gets inversed pwower since it's facing the opposite way
    * 
    ******************************************************************************************/
    public void setDriveMotorPower(double power) {
        power = MathUtil.clamp(power, -1, 1);     
        driveMotor.set(power);
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
        SmartDashboard.putNumber(rotateMotor.getDeviceId() + "Rotation Absolute Encoder", getAdjustedAbsoluteEncoder() );
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

    public void displayPowerAndVelocity() {
        System.out.println(driveMotor.getDeviceId() + " Power " + prevPower + " Velocity " + getDriveVelocity() + " Ratio " + prevPower/getDriveVelocity());
    }
}
// End of the SwerveModule class