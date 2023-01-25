package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;

public class Test {

    private valveState       solenoidState;
    private DoubleSolenoid   solenoid;
    private CANSparkMax      motor;
    private RelativeEncoder  encoder;
    private RelativeEncoder  alternateEncoder;
    private AbsoluteEncoder  absoluteEncoder;
    
    // Prevent robot from creating duplicate objects if testInit() is ran multiple times
    private boolean pneumaticsInitialized;
    private boolean motorInitialized;
    private boolean encoderInitialized;
    private boolean alternateEncoderInitialized;
    private boolean absoluteEncoderInitialized;

    private enum valveState{
        OPEN,
        CLOSED
    }
    
    public Test() {
        // Set the solenoid state to open
        solenoidState = valveState.OPEN;
        
        // Set all initialization trackers to false
        pneumaticsInitialized       = false;
        motorInitialized            = false;
        encoderInitialized          = false;
        alternateEncoderInitialized = false;
        absoluteEncoderInitialized  = false;
    }

    public void initPnuematics(int solenoidIDForward, int solenoidIDReverse) {   
        if (!pneumaticsInitialized) {
            SmartDashboard.putBoolean("Solenoid extended", false);     
            solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, solenoidIDForward, solenoidIDReverse);
        }
    }

    public void initMotor(int motorID, MotorType motorType) {    
        if (!motorInitialized) {
            SmartDashboard.putNumber("Motor power", 0);
            motor = new CANSparkMax(motorID, motorType);
        }
    }

    public void initEncoder() {
        if (!encoderInitialized) {
            SmartDashboard.putNumber("Encoder ticks", 0);
            encoder = motor.getEncoder();
        }
    }

    public void initAlternateEncoder() {
        if (!alternateEncoderInitialized) {
            SmartDashboard.putNumber("Alternate encoder ticks", 0);
            alternateEncoder = motor.getAlternateEncoder(1);
        }
    }

    public void initAbsoluteEncoder() {
        if (!absoluteEncoderInitialized) {
            SmartDashboard.putNumber("Absolute encoder ticks", 0);
            absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        }
    }

    public void solenoidPeriodic() {
        boolean solenoidExtended = SmartDashboard.getBoolean("Solenoid extended", false);
        if(solenoidExtended) {
            solenoid.set(Value.kForward);
        }
        else {
            solenoid.set(Value.kReverse);
        }
    }

    public void motorPeriodic() {
        double power = SmartDashboard.getNumber("Motor power", 0);
        motor.set(power);
    }

    public void encoderPeriodic() {
        double ticks = encoder.getPosition();
        SmartDashboard.putNumber("Encoder ticks", ticks);
    }

    public void alternateEncoderPeriodic() {
        double ticks = alternateEncoder.getPosition();
        SmartDashboard.putNumber("Alternate encoder ticks", ticks);
    }

    public void absoluteEncoderPeriodic() {
        System.out.println("Hello");
        double ticks = absoluteEncoder.getPosition();
        SmartDashboard.putNumber("Absolute encoder ticks", ticks);
    }
}
