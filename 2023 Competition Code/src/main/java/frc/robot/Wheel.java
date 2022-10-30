package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

/**
 * Start of class
 */
public class Wheel {
    // Motor Controllers Declaration
    private CANSparkMax           driveMotor;
    private RelativeEncoder       driveEncoder;
    private CANSparkMax           rotateMotor;
    private Drive.WheelProperties name;

    // The sensor is just a 0V to 5V voltage signal that plugs into the analog inputs in the RoboRio, hence the AnalogPotentiometer objects.
    private AnalogPotentiometer rotateMotorSensor;

    private PIDController rotateController;
    
    // CONSTANTS
    private static final int    WHEEL_CURRENT_LIMIT = 120;
    private static final double ticksPerFoot        = 5.65;
    private static final double ticksPerRevolution  = 20; //CHANGE THIS

    private static final double rP = 0.03;

    // Kinematics Variables
    private double xPos     = 0; // ft
    private double xVel     = 0; // ft/s
    private double prevXVel = 0; // ft/s
    private double xAcc     = 0; // ft/s/s

    private double yPos     = 0; // ft
    private double yVel     = 0; // ft/s
    private double prevYVel = 0; // f/s
    private double yAcc     = 0; // ft/s/s

    /**
     * Wheels Constructor
     * @param driveMotorID
     * @param rotateMotorID
     * @param rotateMotorSensorID
     * @param offsetDegrees
     * @param motorName
     */
    public Wheel(int driveMotorID, int rotateMotorID, int rotateMotorSensorID, double offsetDegrees, Drive.WheelProperties motorName) {
        // Motor Controllers Instantiation
        this.driveMotor   = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.driveEncoder = driveMotor.getEncoder();

        driveEncoder.setPositionConversionFactor((1/60) * (ticksPerRevolution) * (1/ticksPerFoot)); //Minutes --> seconds, revolutions --> ticks, ticks --> feet

        this.rotateMotor  = new CANSparkMax(rotateMotorID, MotorType.kBrushed);
        this.name         = motorName;

        // Adds a current limit and sets the motor mode
        this.driveMotor.setSmartCurrentLimit(WHEEL_CURRENT_LIMIT);
        this.driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.rotateMotor.setSmartCurrentLimit(WHEEL_CURRENT_LIMIT);
        this.rotateMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Rotate Sensor Instantiation
        rotateMotorSensor = new AnalogPotentiometer(rotateMotorSensorID, -360, offsetDegrees);

        //PID Controller
        rotateController = new PIDController(rP, 0, 0);
        rotateController.enableContinuousInput(-180, 180);
        rotateController.setTolerance(4);
    }

    /****************************************************************************************** 
    *
    *    rotateAndDrive()
    *    Indivudally rotates each wheel to a set target angle and powers the drive motor 
    * 
    ******************************************************************************************/
    public int rotateAndDrive(double targetWheelAngle, double drivePower, boolean teleop) {
        double currWheelAngle;
        double rotatePower;

        currWheelAngle = getRotateMotorPosition();

        // If the distance to the target wheel angle is shorter when flipped 180 degrees, then flip target angle and power
        if ( Math.abs(targetWheelAngle - currWheelAngle)   >   Math.abs( normalizeAngle(targetWheelAngle + 180) - currWheelAngle)) {
            targetWheelAngle *= -1;
            drivePower       *= -1;
        }

        rotatePower = rotateController.calculate(currWheelAngle, normalizeAngle(targetWheelAngle));

        /** FOR OLD ROBOT (victor SP):
         * If PID output is positive you want to rotate the wheel clockwise
         * In order to rotate clockwise, a negative power is required
         * And vise-versa.
         * 
         * THE NEW ROBOT USES SPARK MAXES FOR THE ROTATE MOTORS
         * Positive power --> clockwise
         */
        setRotateMotorPower(rotatePower);
        setDriveMotorPower(drivePower);

        // Kinematics updates
        prevXVel = xVel;
        prevYVel = yVel;

        xVel = Math.cos(Math.toRadians(currWheelAngle)) * driveEncoder.getVelocity();
        yVel = Math.sin(Math.toRadians(currWheelAngle)) * driveEncoder.getVelocity();

        // Factors in 50 iterations per second
        xPos += xVel / 50; 
        yPos += yVel / 50;

        xAcc = (xVel - prevXVel) * 50;
        yAcc = (yVel - prevYVel) * 50;

        //The return value is only used in auto
        if (rotateController.atSetpoint()) {
            return Robot.DONE;
        }
        else {
            return Robot.CONT;
        }
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

        // Wheels always go forward. To go reverse, rotate wheels
        if ((name == Drive.WheelProperties.FRONT_LEFT_WHEEL) || 
            (name == Drive.WheelProperties.REAR_LEFT_WHEEL))    {
            driveMotor.set(power * -1);
        } 
        else {
            driveMotor.set(power);
        }
    }

    /****************************************************************************************** 
    *
    *    normalizeAngle()
    *    Takes an input of degrees and brings it from (0 to 360) to (-180 to 180)
    * 
    ******************************************************************************************/
    private double normalizeAngle(double degrees){
        double adjustedValue = degrees;

        if ((adjustedValue >= -180) && (adjustedValue <= 180)) {
            // Does nothing to adjustedValue
        }
        else if(adjustedValue > 180) {
            // Makes all values greater than 180 less than it
            adjustedValue -= 360;
        }
        else if(adjustedValue < -180) {
            // Makes all values less than -180 greater than it
            adjustedValue += 360;
        }
        
        return adjustedValue;
    }

    /****************************************************************************************** 
    *
    *    getRotateMotorPosition()
    *    Returns rotate motor sensor values in a range from -180 to 180
    * 
    ******************************************************************************************/
    public double getRotateMotorPosition() {
        double rawAngle = rotateMotorSensor.get();
        double adjustedValue = normalizeAngle(rawAngle);

        return adjustedValue;
    }

    /****************************************************************************************** 
    *
    *    getEncoderValue()
    *    Returns the encoder value of the drive motor
    * 
    ******************************************************************************************/
    public double getEncoderValue() {
        double tempValue = driveEncoder.getPosition();

        if ((name.equals(Drive.WheelProperties.FRONT_LEFT_WHEEL)) || 
            (name.equals(Drive.WheelProperties.REAR_LEFT_WHEEL)))    {
            tempValue *= -1;
        } 
        return tempValue;
    }

    /****************************************************************************************** 
    *
    *    getters for kinematics
    * 
    ******************************************************************************************/
    public double getXPos() {
        return xPos;
    }
    public double getXVel() {
        return xVel;
    }
    public double getXAcc() {
        return xAcc;
    }

    public double getYPos() {
        return xPos;
    }
    public double getYVel() {
        return xVel;
    }
    public double getYAcc() {
        return xAcc;
    }

    /****************************************************************************************** 
    *
    *    setCoastMode()
    *    Returns the encoder value of the drive motor
    * 
    ******************************************************************************************/
    public void setCoastMode() {
        this.driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        this.rotateMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    /****************************************************************************************** 
    *
    *    testWheelAngle()
    *    Returns the direct reading of the rotate motor sensor
    * 
    ******************************************************************************************/
    public double testWheelAngle() {
        return this.getRotateMotorPosition();
    }

}
// End of the Wheel Class