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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Start of the SwerveModule class
 */
public class SwerveModule {
    private CANSparkMax            driveMotor;
    private CANSparkMax            rotateMotor;
    private RelativeEncoder        driveEncoder;
    private RelativeEncoder        rotateEncoder;
    private AbsoluteEncoder        absoluteEncoder;
    private PIDController          driveMotorController;
    private PIDController          rotateMotorController;
    private SimpleMotorFeedforward driveFeedForward;

    private final int    MOTOR_CURRENT_LIMIT   = 80;
    private final double DRIVE_P               = 0.05;
    private final double DRIVE_I               = 0;
    private final double DRIVE_D               = 0;
    private final double ROTATE_P              = 0.2;
    private final double ROTATE_I              = 0;
    private final double ROTATE_D              = 0;
    private final double STATIC_GAIN           = 0;
    private final double VELOCITY_GAIN         = 0.33;
    private final double WHEEL_RADIUS_METERS   = 0.0762;
    private final double WHEEL_ROTATION_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;
    private final double ROTATIONS_PER_TICK    = 1 / 5.5;
    private final double CONVERSION_FACTOR     = WHEEL_ROTATION_METERS * ROTATIONS_PER_TICK; // Meters per tick

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

        driveMotorController  = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D);
        driveMotorController.enableContinuousInput(-Math.PI, Math.PI);

        driveFeedForward      = new SimpleMotorFeedforward(STATIC_GAIN, VELOCITY_GAIN);

        driveEncoder.setVelocityConversionFactor(CONVERSION_FACTOR);
        driveEncoder.setPositionConversionFactor(CONVERSION_FACTOR);
    }

    /**
     * 
     * @param desiredState
     */
    //  TJM maybe add boolean for optimization control?
    public void move(SwerveModuleState desiredState) {
        // Optimizes the wheel movements
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(getAdjustedAbsoluteEncoder()));

        // Calculates the rotate power
        double currentAngle = getAdjustedAbsoluteEncoder();
        double targetAngle  = optimizedState.angle.getRadians();
        targetAngle = MathUtil.angleModulus(targetAngle);
        double rotatePower  = rotateMotorController.calculate(currentAngle, targetAngle);

        //  TJM  getVelocity() gives you rotations per minute
        //  the targetSpeed if desaturated should give you a value -1 to 1 (-6 to 6 right now)
        //  So not clear if this is working properly
        double currentSpeed = driveEncoder.getVelocity();
        double targetSpeed  = desiredState.speedMetersPerSecond;
        double feedForward  = driveFeedForward.calculate(targetSpeed);
        double pidError     = driveMotorController.calculate(currentSpeed, targetSpeed);
        
        // Sets motor powers
        driveMotor.set(feedForward + pidError);
        rotateMotor.set(rotatePower);
    }

    /*
     * Helper functions
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(rotateEncoder.getPosition()));
    }
    
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
    // TJM Maybe getWheelRotationRadians()  ?
    public double getAdjustedAbsoluteEncoder() {
        // TJM what is negative sign doing here?
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