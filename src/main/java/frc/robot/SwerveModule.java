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
import edu.wpi.first.math.util.Units;
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
    private RelativeEncoder        rotateEncoder;
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

    // Constructor
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
        absoluteEncoder.setPositionConversionFactor(1);
        absoluteEncoder.setInverted(false);
        driveEncoder.setPosition(0);

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
     * 
     * @param desiredState
     */
    //  TJM maybe add boolean for optimization control?
    public void move(SwerveModuleState desiredState) {
        // Optimizes the wheel movements
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(getAdjustedAbsoluteEncoder()));

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

        //System.out.println("Target Angle: " + targetAngle + " Target Speed: " + targetSpeed);
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

    /*
     * Helper functions
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d( getAdjustedAbsoluteEncoder() ));
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
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * 
     * @return
     */
    public double getRotateEncoder() {
        return rotateEncoder.getPosition();
    }

    public boolean rotateControllerAtSetpoint() {
        return rotateMotorController.atSetpoint();
    }

    /**
     * Changes the encoder's range to -pi to pi
     * 
     * @return adjustedRadian
     */
    // TJM Maybe getWheelRotationRadians()  ?
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

    public void displayPowerAndVelocity() {
        System.out.println(driveMotor.getDeviceId() + " Power " + prevPower + " Velocity " + getDriveVelocity() + " Ratio " + prevPower/getDriveVelocity());
    }
}

// End of the SwerveModule class