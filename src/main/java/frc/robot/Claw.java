package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import frc.robot.Controls.ClawState;

public class Claw {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    private DigitalInput button;

    public Claw() {
        leftMotor  = new CANSparkMax(5, MotorType.kBrushless);
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor = new CANSparkMax(6, MotorType.kBrushless);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.follow(leftMotor, true);

        button = new DigitalInput(0);
    }

    public void setIntake(double power) {
        leftMotor.set(power);
    }

    public void startIntake() {
        leftMotor.set(0.08);
    }

    public void endIntake() {
        leftMotor.set(0);
    }

    public void startOutput() {
        leftMotor.set(-0.2);
    }

    public boolean getButtonPressed() {
        return !button.get();
    }
}
