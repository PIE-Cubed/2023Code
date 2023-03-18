package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Claw {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private RelativeEncoder leftEncoder;

    private int resistanceCount = 0;

    public Claw() {
        leftMotor  = new CANSparkMax(5, MotorType.kBrushless);
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor = new CANSparkMax(6, MotorType.kBrushless);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.follow(leftMotor, true);

        leftEncoder = leftMotor.getEncoder();
    }

    public void setIntake(double power) {
        leftMotor.set(power);
    }

    public void startIntake() {
        leftMotor.set(0.08);
        resistanceCount = 0;
    }

    public void endIntake() {
        leftMotor.set(0);
    }

    public double getVelocity() {
        return leftEncoder.getVelocity();
    }

    public boolean resistance() {
        double power = leftMotor.get();

        if (Math.abs(power) > 0) {
            if (Math.abs(leftEncoder.getVelocity()) < 200) {
                resistanceCount ++;
            }
            else {
                resistanceCount = 0;
            } 
            return (resistanceCount > 10);
        }
        return false;
    }
}
