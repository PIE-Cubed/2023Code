package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;

public class Arm {
    // Object Creation
    private CANSparkMax shoulderMotor; 
    private CANSparkMax elbowMotor;

    private RelativeEncoder shoulderEncoder;
    private RelativeEncoder elbowEncoder;

    private PIDController shoulderController;
    private PIDController elbowContorller;

    private ArmFeedforward shoulderFeedForward;
    private ArmFeedforward elbowFeedForward;

    // Controller Constants
    private final double sP = 0; // Shoulder PID Proportionality
    private final double sI = 0; // Shoulder PID Integral
    private final double sD = 0; // Shoulder PID Derivative
    private final double eP = 0; // Elbow PID Proportionality
    private final double eI = 0; // Elbow PiD Integral
    private final double eD = 0; // Elbow PID Derivative

    private final double sS = 0; // Shoulder ArmFeedFoward Static Coefficient
    private final double sG = 0; // Shoulder ArmFeedFoward Gravity Coefficient
    private final double sV = 0; // Shoulder ArmFeedFoward Velocity Coefficient
    private final double eS = 0; // Elbow ArmFeedFoward Static Coefficient
    private final double eG = 0; // Elbow ArmFeedFoward Gravity Coefficient
    private final double eV = 0; // Elbow ArmFeedFoward Velocity Coefficient

    public Arm() {
        shoulderMotor       = new CANSparkMax(0, MotorType.kBrushless);
        elbowMotor          = new CANSparkMax(1, MotorType.kBrushless);

        shoulderEncoder     = shoulderMotor.getEncoder();
        elbowEncoder        = elbowMotor.getEncoder();

        shoulderController  = new PIDController(sP, sI, sD);
        elbowContorller     = new PIDController(eP, eI, eD);

        shoulderFeedForward = new ArmFeedforward(sS, sG, sV);
        elbowFeedForward    = new ArmFeedforward(eS, eG, eV);
    }

    public void shoulderPower(double power) {
        shoulderMotor.set(power);
    }

    public void elbowPower(double power) {
        elbowMotor.set(power);
    }
}

