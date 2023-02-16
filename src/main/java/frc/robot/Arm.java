package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;

public class Arm {
    // Object Creation
    private CANSparkMax baseMotor;
    private CANSparkMax middleMotor; 
    private CANSparkMax endMotor;

    //private RelativeEncoder baseEncoder;
    //private RelativeEncoder middleEncoder;
    //private RelativeEncoder endEncoder;

    private AbsoluteEncoder baseAbsoluteEncoder;
    private AbsoluteEncoder middleAbsoluteEncoder;
    private AbsoluteEncoder endAbsoluteEncoder;

    // Used to calculate torque on motors
    private Translation2d baseJointLocation   = new Translation2d(0, 0);
    private Translation2d middleJointLocation = new Translation2d(0, 0);
    private Translation2d endJointLocation    = new Translation2d(0, 0);

    // Constants - need to measure
    private final double LENGTH_BASE   = 0.5;
    private final double LENGTH_MIDDLE = 0.5;
    private final double LENGTH_END    = 0.5;


    public Arm() {
        baseMotor   = new CANSparkMax(5, MotorType.kBrushless);
        middleMotor = new CANSparkMax(6, MotorType.kBrushless);
        endMotor    = new CANSparkMax(7, MotorType.kBrushless);

        baseMotor.setIdleMode(IdleMode.kBrake);
        middleMotor.setIdleMode(IdleMode.kBrake);
        endMotor.setIdleMode(IdleMode.kBrake);

        //baseEncoder   = baseMotor.getEncoder();
        //middleEncoder = middleMotor.getEncoder();
        //endEncoder    = endMotor.getEncoder();

        baseAbsoluteEncoder   = baseMotor.getAbsoluteEncoder(Type.kDutyCycle);
        middleAbsoluteEncoder = middleMotor.getAbsoluteEncoder(Type.kDutyCycle);
        endAbsoluteEncoder    = endMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // TBD
        baseAbsoluteEncoder.setPositionConversionFactor(1);
        middleAbsoluteEncoder.setPositionConversionFactor(1);
        endAbsoluteEncoder.setPositionConversionFactor(1);
    }

    // Determines the angle of each joint. Wrist angle is relative to the floor in radians
    public double[] getJointAngles(double reachAngle, double reachDistance, double wristAngle) {
        // Total X and Y from base to end of claw
        double reachX = reachDistance * Math.cos(reachAngle);
        double reachY = reachDistance * Math.sin(reachAngle);

        // X and Y from base to joint 2
        double joint2X   = reachX - (LENGTH_END * Math.cos(wristAngle));
        double joint2Y   = reachY - (LENGTH_END * Math.sin(wristAngle));
        double joint2Hyp = Math.sqrt( Math.pow(joint2X, 2) + Math.pow(joint2Y, 2) );

        // Can now consider triangle between Joint 0, 1, and 2.
        // Using Heron's formula, can find area of this triangle
        double s = (LENGTH_BASE + LENGTH_MIDDLE + joint2Hyp) / 2;
        double area = Math.sqrt(s * (s - LENGTH_BASE) * (s - LENGTH_MIDDLE) * (s - joint2Hyp));

        // Using area, can find height from base (line from J0 to J2) to J1
        double height = 2 * area / joint2Hyp;

        // Using value of height, we have 2 right triangles that can be solved for angle 1 and 2
        // NEED TO DO
    }

    public void setBasePower(double power) {
        baseMotor.set(power);
    }

    public void setMiddlePower(double power) {
        middleMotor.set(power);
    }

    public void setEndPower(double power) {
        endMotor.set(power);
    }

    public double getBaseRotation() {
        return baseAbsoluteEncoder.getPosition();
    }

    public double getMiddleRotation() {
        return middleAbsoluteEncoder.getPosition();
    }

    public double getEndRotation() {
        return endAbsoluteEncoder.getPosition();
    }
}

