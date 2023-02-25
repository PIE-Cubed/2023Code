package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Controls.ArmStates;
import frc.robot.Controls.Objects;

public class Arm {
    // Object Creation
    private CANSparkMax baseMotor;
    private CANSparkMax middleMotor; 
    private CANSparkMax endMotor;

    private AbsoluteEncoder baseAbsoluteEncoder;
    private AbsoluteEncoder middleAbsoluteEncoder;
    private AbsoluteEncoder endAbsoluteEncoder;

	private DoubleSolenoid  claw;
	private final int PNEU_CONTROLLER_ID = 1;

	private double  printCount = 0;
	private int     step       = 1;
	private boolean firstTime  = false;


    // Constants - need to verify joint limits
    private final double LENGTH_BASE   = 0.5588;
    private final double LENGTH_MIDDLE = 0.53848;
    private final double LENGTH_END    = 0.6096; //0.4318
	
	private final double ANGLE_1_MIN   = Math.PI / 6;
	private final double ANGLE_1_MAX   = Math.PI;
	private final double ANGLE_2_MIN   = -5 * Math.PI / 6;
	private final double ANGLE_2_MAX   = 5  * Math.PI / 6;
	private final double ANGLE_3_MIN   = -5 * Math.PI / 6;
	private final double ANGLE_3_MAX   = Math.PI / 2;
	
	// Masses in kg
	private final double BASE_MASS    = 1.9912705;
	private final double MIDDLE_MASS  = 1.44695966;
	private final double END_MASS     = 0.94800805;
	private final double JOINT_2_MASS = 0.48080791;
	private final double JOINT_3_MASS = 0.48080791;
	private final double CONE_MASS    = 0.65203903;
	private final double CUBE_MASS    = 0.07087381;

	// Negative ratio to counter the force of gravity
	private final double BASE_TORQUE_TO_POWER   = -0.01;
	private final double MIDDLE_TORQUE_TO_POWER = -0.0175;
	private final double END_TORQUE_TO_POWER    = -0.5;

	// PIDs
	private final double p1 = 0.4; //0.16 without cone
	private final double p2 = 0.3;
	private final double p3 = 0.3;

	private final double BASE_TOLERANCE   = Math.PI / 45; // Base angle has the most effects, needs to be tight
	private final double MIDDLE_TOLERANCE = Math.PI / 25;
	private final double END_TOLERANCE    = Math.PI / 25;

	private PIDController basePID;  
	private PIDController middlePID;
	private PIDController endPID;
	
	// Constructor
    public Arm() {
        baseMotor   = new CANSparkMax(5, MotorType.kBrushless);
        middleMotor = new CANSparkMax(7, MotorType.kBrushless);
        endMotor    = new CANSparkMax(6, MotorType.kBrushless);

		claw = new DoubleSolenoid(PNEU_CONTROLLER_ID, PneumaticsModuleType.REVPH, 15, 0);

        baseMotor.setIdleMode(IdleMode.kBrake);
        middleMotor.setIdleMode(IdleMode.kBrake);
        endMotor.setIdleMode(IdleMode.kBrake);

		baseMotor.setInverted(true);
		middleMotor.setInverted(false);

        baseAbsoluteEncoder   = baseMotor.getAbsoluteEncoder(Type.kDutyCycle);
        middleAbsoluteEncoder = middleMotor.getAbsoluteEncoder(Type.kDutyCycle);
        endAbsoluteEncoder    = endMotor.getAbsoluteEncoder(Type.kDutyCycle);

        baseAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        middleAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        endAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI);
		baseAbsoluteEncoder.setVelocityConversionFactor(2 * Math.PI);
        middleAbsoluteEncoder.setVelocityConversionFactor(2 * Math.PI);
        endAbsoluteEncoder.setVelocityConversionFactor(2 * Math.PI);
		endAbsoluteEncoder.setInverted(true);
		
		basePID   = new PIDController(p1, 0, 0);
		middlePID = new PIDController(p2, 0, 0);
		endPID    = new PIDController(p3, 0, 0);

		basePID.setTolerance(BASE_TOLERANCE);
		middlePID.setTolerance(MIDDLE_TOLERANCE);
		endPID.setTolerance(END_TOLERANCE);
    }

	public int toGrabPosition() {
		if (endAbsoluteEncoder.getPosition() > 3.85 || endAbsoluteEncoder.getPosition() < 1) {
			endMotor.set(0.3);
			return Robot.CONT;
		}
		else {
			endMotor.set(0);
			return Robot.DONE;
		}
	}

	public int toRestPosition() {

		if (firstTime) {
			step = 1;
			firstTime = false;
		}

		if (step == 1) {
			boolean done = true;

			if (endAbsoluteEncoder.getPosition() < 5.8 && endAbsoluteEncoder.getPosition() > 1) {
				done = false;
				endMotor.set(-0.4);
			}
			else {
				endMotor.set(0);
			}

			if (done) {
				step = 2;
			}
		}

		if (step == 2) {
			boolean done = true;

			if (!(middleAbsoluteEncoder.getPosition() > 5.8 || middleAbsoluteEncoder.getPosition() < 0.5)) {
				middleMotor.set(-0.75);
				done = false;
			}
			else {
				middleMotor.set(0);
			}
	
			if (!(baseAbsoluteEncoder.getPosition() > 6.0 || baseAbsoluteEncoder.getPosition() < 0.5)) {
				baseMotor.set(-0.5);
				done = false;
			}
			else {
				baseMotor.set(0);
			}

			if (done) {
				return Robot.DONE;
			}
		}

		return Robot.CONT;
	}

	public int toMidCone() {
		if (firstTime) {
			step = 1;
			firstTime = false;
		}

		if (step == 1) {
			boolean done = true;

			if (middleAbsoluteEncoder.getPosition() > 4.55 || middleAbsoluteEncoder.getPosition() < 1) {
				middleMotor.set(0.75);
				done = false;
			}
			else {
				middleMotor.set(0);
			}
	
			if (baseAbsoluteEncoder.getPosition() < 0.52 || baseAbsoluteEncoder.getPosition() > 5.5) {
				baseMotor.set(0.5);
				done = false;
			}
			else {
				baseMotor.set(0);
			}

			if (done) {
				step = 2;
			}
		}

		if (step == 2) {
			if (endAbsoluteEncoder.getPosition() > 2.84 || endAbsoluteEncoder.getPosition() < 1) {
				endMotor.set(0.4);
			}
			else {
				endMotor.set(0);
				return Robot.DONE;
			}
		}

		return Robot.CONT;
	}

	public int toMidCube() {
		if (firstTime) {
			step = 1;
			firstTime = false;
		}

		if (step == 1) {
			if (middleAbsoluteEncoder.getPosition() > 5.3 || middleAbsoluteEncoder.getPosition() < 1) {
				middleMotor.set(0.75);
			}
			else {
				middleMotor.set(0);
				step = 2;
			}
		}

		if (step == 2) {
			if (endAbsoluteEncoder.getPosition() > 3.02 || endAbsoluteEncoder.getPosition() < 1) {
				endMotor.set(0.4);
			}
			else {
				endMotor.set(0);
				return Robot.DONE;
			}
		}

		return Robot.CONT;
	}

	public int toTopCone() {
		if (firstTime) {
			step = 1;
			firstTime = false;
		}

		if (step == 1) {
			boolean done = true;

			if (middleAbsoluteEncoder.getPosition() > 3.32 || middleAbsoluteEncoder.getPosition() < 1) {
				middleMotor.set(0.75);
				done = false;
			}
			else {
				middleMotor.set(0);
			}
	
			if (baseAbsoluteEncoder.getPosition() < 1.61 || baseAbsoluteEncoder.getPosition() > 5.5) {
				baseMotor.set(0.5);
				done = false;
			}
			else {
				baseMotor.set(0);
			}

			if (done) {
				step = 2;
			}
		}

		if (step == 2) {
			if (endAbsoluteEncoder.getPosition() > 3.35 || endAbsoluteEncoder.getPosition() < 1) {
				endMotor.set(0.4);
			}
			else {
				endMotor.set(0);
				return Robot.DONE;
			}
		}

		return Robot.CONT;
	}

	public int toTopCube() {
		if (firstTime) {
			step = 1;
			firstTime = false;
		}

		if (step == 1) {
			boolean done = true;

			if (middleAbsoluteEncoder.getPosition() > 4.62 || middleAbsoluteEncoder.getPosition() < 1) {
				middleMotor.set(0.75);
				done = false;
			}
			else {
				middleMotor.set(0);
			}
	
			if (baseAbsoluteEncoder.getPosition() < 0.83 || baseAbsoluteEncoder.getPosition() > 5.5) {
				baseMotor.set(0.5);
				done = false;
			}
			else {
				baseMotor.set(0);
			}

			if (done) {
				step = 2;
			}
		}

		if (step == 2) {
			if (endAbsoluteEncoder.getPosition() > 3.57 || endAbsoluteEncoder.getPosition() < 1) {
				endMotor.set(0.4);
			}
			else {
				endMotor.set(0);
				return Robot.DONE;
			}
		}

		return Robot.CONT;
	}

	public void resetArmState() {
		firstTime = true;
		step = 1;
	}

	public void powerEnd(double power) {
		double[] restArmPowers = restArmPowers();

		// If arm is folded in, base and middle don't need powers to hold their position
		if (Controls.armState == ArmStates.REST || Controls.armState == ArmStates.GRAB) {
			setMiddlePower(0);
			setEndPower(0);
		}
		else {
			setMiddlePower(restArmPowers[1]);
			setEndPower(restArmPowers[2]);
		}
		setEndPower(power + restArmPowers[3]);
	}

	public int setArmAngles(double baseAngle, double middleAngle, double endAngle) {
		// Target joint angles
		double[] targetJointAngles  = {baseAngle, middleAngle, endAngle};
		for (int i = 0; i < 3; i++) {
			targetJointAngles[i] = MathUtil.angleModulus(targetJointAngles[i]);
		}
		
		basePID.setSetpoint(targetJointAngles[0]);
		middlePID.setSetpoint(targetJointAngles[1]);
		endPID.setSetpoint(targetJointAngles[2]);

		// Using current joint angles to get PID powers
		double[] currentJointAngles = {
			getAdjustedBaseAngle(),
			getAdjustedMiddleAngle(),
			getAdjustedEndAngle()
		};
		double basePower   = basePID.calculate(currentJointAngles[0], targetJointAngles[0]);
		double middlePower = middlePID.calculate(currentJointAngles[1], targetJointAngles[1]);
		double endPower    = endPID.calculate(currentJointAngles[2], targetJointAngles[2]);

		// Combining PID power and feed forward to set motor powers
		double[] feedForward = restArmPowers();

		setBasePower(basePower + feedForward[0]);
		setMiddlePower(middlePower + feedForward[1]);
		setEndPower(endPower + feedForward[2]);

		// Checking if all joints are at setpoint
		if (basePID.atSetpoint() && middlePID.atSetpoint() && endPID.atSetpoint()) {
			return Robot.DONE;
		}
		return Robot.CONT;
	}

	public double[] restArmPowers() {
		// Just base for now
		double q1 = baseAbsoluteEncoder.getPosition();
		double q2 = middleAbsoluteEncoder.getPosition();
		double q3 = endAbsoluteEncoder.getPosition();

		double torque1 = torqueJoint1(q1, q2, q3);
		double power1 = torque1 * BASE_TORQUE_TO_POWER;

		double torque2 = torqueJoint2(q1, q2, q3);
		double power2 = torque2 * MIDDLE_TORQUE_TO_POWER;

		double torque3 = torqueJoint3(q1, q2, q3);
		double power3 = torque3 * END_TORQUE_TO_POWER;

		double[] results = {power1, power2, power3};
		return results;
	}

	// Positive torque --> toward positive angle. If arm is back to positive X, it will have a negative torque, therefore, X is multiplied by -1
	// kg * m^2/s^2
    public double torqueJoint1(double q1, double q2, double q3) {
		// Note: gravity lever arm is simply the X distance from the joint
		double joint2X       = LENGTH_BASE * Math.cos(q1);
		double joint3X       = joint2X + LENGTH_MIDDLE * Math.cos(q1 + q2);
		double endX          = joint3X + LENGTH_END * Math.cos(q1 + q2 + q3);
		
		double baseCenterX   = joint2X / 2;
		double middleCenterX = (joint2X + joint3X) / 2;
		double endCenterX    = (joint3X + endX) / 2;
		
		double joint2Torque  = JOINT_2_MASS * -1 * joint2X;
		double joint3Torque  = JOINT_3_MASS * -1 * joint3X;

		double objectMass = 0;
		if (Controls.currentObject == Objects.CONE) {
			objectMass = CONE_MASS;
		}
		else if (Controls.currentObject == Objects.CUBE) {
			objectMass = CUBE_MASS;
		}

		double clawTorque    = objectMass   * -1 * endX; // Implement cone/cube mass
		
		double baseTorque    = BASE_MASS    * -1 * baseCenterX;
		double middleTorque  = MIDDLE_MASS  * -1 * middleCenterX;
		double endTorque     = END_MASS     * -1 * endCenterX;
		
		return joint2Torque + joint3Torque + clawTorque + baseTorque + middleTorque + endTorque;
	}
	
	public double torqueJoint2(double q1, double q2, double q3) {
		// All distances are relative to Joint 2
		double joint3X       = LENGTH_MIDDLE * Math.cos(q1 + q2);
		double endX          = joint3X + LENGTH_END * Math.cos(q1 + q2 + q3);
		
		double middleCenterX = joint3X / 2;
		double endCenterX    = (joint3X + endX) / 2;
		
		double joint3Torque  = JOINT_3_MASS * -1 * joint3X;

		double objectMass = 0;
		if (Controls.currentObject == Objects.CONE) {
			objectMass = CONE_MASS;
		}
		else if (Controls.currentObject == Objects.CUBE) {
			objectMass = CUBE_MASS;
		}

		double clawTorque    = objectMass   * -1 * endX; // Implement cone/cube mass
		
		double middleTorque  = MIDDLE_MASS  * -1 * middleCenterX;
		double endTorque     = END_MASS     * -1 * endCenterX;
		
		return joint3Torque + clawTorque + middleTorque + endTorque;
	}
	
	public double torqueJoint3(double q1, double q2, double q3) {
		// All distances are relative to Joint 3
		double endX          = LENGTH_END * Math.cos(q1 + q2 + q3);
		double endCenterX    = endX / 2;

		double objectMass = 0;
		if (Controls.currentObject == Objects.CONE) {
			objectMass = CONE_MASS;
		}
		else if (Controls.currentObject == Objects.CUBE) {
			objectMass = CUBE_MASS;
		}
		
		double clawTorque    = objectMass * -1 * (endX - 0.15); // Implement cone/cube mass
		double endTorque     = END_MASS   * -1 * endCenterX;
		
		return clawTorque + endTorque;
	}
	
	// kg * m^2
	public double rotationalInertiaJoint1(double q1, double q2, double q3) {
		// Integral representation of inertia of horinzontal line is m * integral from r1 to r2 of r^2
		// Solving integral results in m/3 * ((r2)^3 - (r1)^3)
		double joint2X       = LENGTH_BASE * Math.cos(q1);
		double joint2Y       = LENGTH_BASE * Math.sin(q1);
		double joint2R       = LENGTH_BASE;
		
		double joint3X       = joint2X + LENGTH_MIDDLE * Math.cos(q1 + q2);
		double joint3Y       = joint2Y + LENGTH_MIDDLE * Math.sin(q1 + q2);
		double joint3R       = Math.hypot(joint3X, joint3Y);
		
		double endX          = joint3X + LENGTH_END * Math.cos(q1 + q2 + q3);
		double endY          = joint3Y + LENGTH_END * Math.sin(q1 + q2 + q3);	
		double endR          = Math.hypot(endX, endY);
		
		// Torque of point is mr^2
		double joint2Inertia = JOINT_2_MASS * Math.pow(joint2R, 2);
		double joint3Inertia = JOINT_3_MASS * Math.pow(joint3R, 2);
		double clawInertia    = 0 * Math.pow(endR, 2); // Implement cone/cube mass

		// Torque of line uses integral
		double baseInertia   = BASE_MASS   * 1/3 * (Math.pow(joint2R, 3) - 0);
		double middleInertia = MIDDLE_MASS * 1/3 * (Math.pow(joint3R, 3) - Math.pow(joint2R, 3));
		double endInertia    = END_MASS    * 1/3 * (Math.pow(endR, 3)    - Math.pow(joint3R, 3));
		
		return joint2Inertia + joint3Inertia + clawInertia + baseInertia + middleInertia + endInertia;
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

	/*
	 * Methods to get angles of arms
	 * Offset so 0 is backwards on base, and straight on the middle and end
	 * Wrapped to -pi to pi
	 */
    public double getAdjustedBaseAngle() {
        return MathUtil.angleModulus(baseAbsoluteEncoder.getPosition() + 0);
    }

    public double getAdjustedMiddleAngle() {
        return MathUtil.angleModulus(middleAbsoluteEncoder.getPosition() + 0);
    }

    public double getAdjustedEndAngle() {
        return MathUtil.angleModulus(endAbsoluteEncoder.getPosition() + 0);
    }


	public void openClaw() {
		claw.set(Value.kReverse);
	}

	public void closeClaw() {
		claw.set(Value.kForward);
	}

	// Test functions
	public void testAbsEncoders() {
		if (printCount % 15 == 0) {
			System.out.println("Base:" + baseAbsoluteEncoder.getPosition() + " Middle:" + middleAbsoluteEncoder.getPosition() + " End:" + endAbsoluteEncoder.getPosition());
		}
		printCount++;
	}

	public void testMiddlePower() {
		if (firstTime) {
			step = 1;
			firstTime = false;
		}

		if (step == 1) {
			boolean done = true;

			if (middleAbsoluteEncoder.getPosition() > 4.55 || middleAbsoluteEncoder.getPosition() < 1) {
				middleMotor.set(0.75);
				done = false;
			}
			else {
				middleMotor.set(0);
			}
	
			if (baseAbsoluteEncoder.getPosition() < 0.52 || baseAbsoluteEncoder.getPosition() > 5.5) {
				baseMotor.set(0.5);
				done = false;
			}
			else {
				baseMotor.set(0);
			}

			if (done) {
				step = 2;
			}
		}
		if (step == 2) {
			if (endAbsoluteEncoder.getPosition() > 2.84 || endAbsoluteEncoder.getPosition() < 1) {
				endMotor.set(0.4);
			}
			else {
				endMotor.set(0);
			}
		}
		
	}

	public void testTorque() {
		double q1 = baseAbsoluteEncoder.getPosition();
		double q2 = middleAbsoluteEncoder.getPosition();
		double q3 = endAbsoluteEncoder.getPosition();

		if (printCount % 15 == 0) {
			System.out.println(" End:" + torqueJoint3(q1, q2, q3) + " Total angle:" + (q1+q2+q3));
		}
		printCount++;
	}
}
