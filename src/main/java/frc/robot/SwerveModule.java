package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Start of the SwerveModule class
 */
public class SwerveModule {
    private CANSparkMax     driveMotor;
    private CANSparkMax     rotateMotor;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder rotateEncoder;
    private AbsoluteEncoder absoluteEncoder;
    private PIDController   rotateMotorController;

    private final double TICKS_PER_METER     = 1;
    private final double TICKS_PER_RADIAN    = 1;
    private final int    MOTOR_CURRENT_LIMIT = 80;
    private final double ROTATE_P = 0.2;
    private final double ROTATE_I = 0;
    private final double ROTATE_D = 0;

    public SwerveModule(int driveID, int rotateID, boolean invertMotor) {
        driveMotor            = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(invertMotor);

        rotateMotor           = new CANSparkMax(rotateID, MotorType.kBrushless);
        rotateMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        rotateMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder          = driveMotor.getEncoder();
        rotateEncoder         = rotateMotor.getEncoder();
        absoluteEncoder       = rotateMotor.getAbsoluteEncoder(Type.kDutyCycle);

        rotateMotorController = new PIDController(ROTATE_P, ROTATE_I, ROTATE_D);
        rotateMotorController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * 
     * @param desiredState
     */
    public void move(SwerveModuleState desiredState) {
        // Optimizes the wheel movements
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(getAdjustedAbsoluteEncoder()));

        // Calculates the rotate power
        double currentAngle = getAdjustedAbsoluteEncoder();
        double targetAngle  = optimizedState.angle.getRadians();
        targetAngle = MathUtil.angleModulus(targetAngle);
        double rotatePower  = rotateMotorController.calculate(currentAngle, targetAngle);
        
        // Sets motor powers
        driveMotor.set(MathUtil.clamp(optimizedState.speedMetersPerSecond, -0.5, 0.5));
        rotateMotor.set(rotatePower);
    }

    /*
     * Helper functions
     */
    /**
     * 
     * @return
     */
    public double getDriveEncoder() {
        return driveEncoder.getPosition();
    }

    /**
     * 
     * @return
     */
    public double getRotateEncoder() {
        return rotateEncoder.getPosition();
    }

    /**
     * Changes the encoder's range to -pi to pi
     * 
     * @return adjustedRadian
     */
    public double getAdjustedAbsoluteEncoder() {
        double rad = -2 * Math.PI * absoluteEncoder.getPosition();

        rad = MathUtil.angleModulus(rad);

        return rad;
    }

    /*
     * Test functions for use with shuffleboard
     */
    public void displayEncoderValues() {
        SmartDashboard.putNumber(driveMotor.getDeviceId() + "Drive Encoder", getDriveEncoder());
        SmartDashboard.putNumber(rotateMotor.getDeviceId() + "Rotate Encoder", getRotateEncoder());
        SmartDashboard.putNumber(rotateMotor.getDeviceId() + "Rotation Absolute Encoder", getAdjustedAbsoluteEncoder());
    }

    public void initDriveMotorSlider() {
        SmartDashboard.putNumber("Drive Motor Power", 0);
    }

    public void initRotateMotorSlider() {
        SmartDashboard.putNumber("Rotate Motor Power", 0);
    }

    public void updateMotorPeriodic() {
        driveMotor.set(SmartDashboard.getNumber("Drive Motor Power", 0));
        rotateMotor.set(SmartDashboard.getNumber("Rotate Motor Power", 0));
    }
}

// End of the SwerveModule class