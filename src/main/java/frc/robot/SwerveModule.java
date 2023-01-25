package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;


public class SwerveModule {
    private CANSparkMax     driveMotor;
    private CANSparkMax     rotateMotor;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder rotateEncoder;
    private AbsoluteEncoder absoluteEncoder;
    private PIDController   rotateMotorController;

    private final double TICKS_PER_METER  = 1;
    private final double TICKS_PER_RADIAN = 1;
    private final double ABS_CONVERSION   = 1;
    private final double ROTATE_P = 0.1;
    private final double ROTATE_I = 0;
    private final double ROTATE_D = 0;

    public SwerveModule(int driveID, int rotateID) {
        driveMotor            = new CANSparkMax(driveID, MotorType.kBrushless);
        rotateMotor           = new CANSparkMax(rotateID, MotorType.kBrushless);
        driveEncoder          = driveMotor.getEncoder();
        rotateEncoder         = rotateMotor.getEncoder();
        absoluteEncoder       = rotateMotor.getAbsoluteEncoder(Type.kDutyCycle);
        rotateMotorController = new PIDController(ROTATE_I, ROTATE_D, ABS_CONVERSION);
    }

    public void move(SwerveModuleState desiredState) {
        driveMotor.set(desiredState.speedMetersPerSecond);
        double currentAngle = absoluteEncoder.getPosition() * ABS_CONVERSION;
        double targetAngle  = desiredState.angle.getRadians();
        double rotatePower  = rotateMotorController.calculate(currentAngle, targetAngle);
        rotateMotor.set(rotatePower);
    }
    
}
