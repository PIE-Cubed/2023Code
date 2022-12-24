// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

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

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax rotateMotor;

    private final RelativeEncoder driveEncoder;
    private final AnalogPotentiometer rotateEncoder;

    private final PIDController driveController = new PIDController(Constants.dP, Constants.dI, Constants.dD);

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private final ProfiledPIDController rotateController =
        new ProfiledPIDController(
            Constants.rotateP, Constants.rotateI, Constants.rotateD,
            new TrapezoidProfile.Constraints(Constants.MAX_MODULE_ROTATE_SPEED, Constants.MAX_MODULE_ROTATE_ACCELERATION));

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorId The channel of the drive motor.
     * @param rotateMotorId The channel of the turning motor.
     * @param absoluteSensorId The channel of the absolute encoder. 
     * @param driveMotorInverted Whether the drive motor is reversed.
     * @param offsetDegrees The absolute encoder offset in degrees.
     */
    public SwerveModule(int driveMotorId, int rotateMotorId, int absoluteSensorId, boolean driveMotorInverted, double offsetDegrees) {
        // Creates the drive motor and encoder
        driveMotor   = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();

        // Creates the rotate motor and encoder
        rotateMotor   = new CANSparkMax(rotateMotorId, MotorType.kBrushless);
        rotateEncoder = new AnalogPotentiometer(absoluteSensorId, -360, offsetDegrees);

        // Invert the motors if necessary
        driveMotor.setInverted(driveMotorInverted);

        // Limit the PID Controller's input range between -pi and pi and set the input to be continuous.
        rotateController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d( degToRad(getRotationPos()) ));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d( degToRad(getRotationPos()) ));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = driveController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = rotateController.calculate(getRotationPos(), state.angle.getDegrees());

        // Calculate the turning motor output from the turning PID controller.
        driveMotor.set(driveOutput);
        rotateMotor.set(turnOutput);
    }

    /**
     * degtoRad()
     * <p>Converts degrees to radians.
     * @param degrees
     * @return radians
     */
    private double degToRad(double degrees) {
        return Units.degreesToRadians(degrees);
    }

    /**
     * getRotationPos()
     * <p>Returns rotate encoder values in a range from -180 to 180
     * @return
     */
    private double getRotationPos() {
        double rawAngle = rotateEncoder.get();

        return normalizeAngle(rawAngle);
    }

    /**
     * normalizeAngle()
     * <p>Takes an input of degrees and brings it from (0 to 360) to (-180 to 180)
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
}

// End of the SwerveModule class