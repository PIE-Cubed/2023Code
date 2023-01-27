package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveModule {
    private CANSparkMax     driveMotor;
    private CANSparkMax     rotateMotor;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder rotateEncoder;
    private AbsoluteEncoder absoluteEncoder;
    private PIDController   rotateMotorController;

    private final double TICKS_PER_METER     = 1;
    private final double TICKS_PER_RADIAN    = 1;
    private final double ABS_CONVERSION      = 1;
    private final int MOTOR_CURRENT_LIMIT = 80;
    private final double ROTATE_P = 0.1;
    private final double ROTATE_I = 0;
    private final double ROTATE_D = 0;

    public SwerveModule(int driveID, int rotateID, boolean invertMotor) {
        driveMotor            = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        driveMotor.setInverted(invertMotor);

        rotateMotor           = new CANSparkMax(rotateID, MotorType.kBrushless);
        rotateMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);

        driveEncoder          = driveMotor.getEncoder();
        rotateEncoder         = rotateMotor.getEncoder();
        absoluteEncoder       = rotateMotor.getAbsoluteEncoder(Type.kDutyCycle);

        rotateMotorController = new PIDController(ROTATE_I, ROTATE_D, ABS_CONVERSION);
        rotateMotorController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void move(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(absoluteEncoder.getPosition()));
        driveMotor.set(optimizedState.speedMetersPerSecond);
        double currentAngle = absoluteEncoder.getPosition() * ABS_CONVERSION;
        double targetAngle  = optimizedState.angle.getRadians();
        double rotatePower  = rotateMotorController.calculate(currentAngle, targetAngle);
        rotateMotor.set(rotatePower);
    }

    /*
     * Helper functions
     */
    public double getDriveEncoder() {
        return driveEncoder.getPosition();
    }

    public double getRotateEncoder() {
        return rotateEncoder.getPosition();
    }

    /*
     * Test functions for use with smart dashboard
     */
    public void displayEncoderValues() {
        SmartDashboard.putNumber("Drive Encoder", getDriveEncoder());
        SmartDashboard.putNumber("Rotate Encoder", getRotateEncoder());
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
