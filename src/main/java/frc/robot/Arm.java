package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
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

	private DigitalInput limitButton0;
	private DigitalInput limitButton9;

	private final int PNEU_CONTROLLER_ID = 1;
	private double printCount = 0;
	private int switchCount = 0;
	
    // Constants
    private final double LENGTH_BASE   = 0.5545;
    private final double LENGTH_MIDDLE = 0.4498;
    private final double LENGTH_END    = 0.5842; // 0.7112 // Center of mass may be closer to joint because claw is lighter than arm
	
	// Joint limits use same angle measurements that are used to calculate physics and kinematics
	private final double ANGLE_1_MIN   = 0.817;
	private final double ANGLE_1_MAX   = Math.PI; // Need to consider if it goes to -pi
	private final double ANGLE_2_MIN   = -Math.PI/2;
	private final double ANGLE_2_MAX   = 2.8;
	private final double ANGLE_3_MIN   = -2.985;
	private final double ANGLE_3_MAX   = Math.PI / 2;

	// Max motor powers
	private final double MAX_END_POWER = 0.7; //0.4 is slow
	
	// Masses in kg - updated for new arm
	private final double BASE_MASS    = 3.175;
	private final double MIDDLE_MASS  = 1.814;
	private final double END_MASS     = 1.633;
	private final double JOINT_2_MASS = 0.395;
	private final double JOINT_3_MASS = 0.671;
	private final double CONE_MASS    = 0.652;
	private final double CUBE_MASS    = 0.071;

	private final double BASE_TORQUE_TO_POWER   = -0.01;
	private final double MIDDLE_TORQUE_TO_POWER = 0.015;
	private final double END_TORQUE_TO_POWER    = -0.035;

	// PIDs
	private final double p1 = 0.4;
	private final double p2 = -1;
	private final double p3 = 1.8;
	private final double i3 = 0;//0.5; 

	private final double BASE_TOLERANCE   = Math.PI / 45; // Base angle has the most effects, needs to be tight
	private final double MIDDLE_TOLERANCE = Math.PI / 25;
	private final double END_TOLERANCE    = Math.PI / 25;

	private PIDController basePID;  
	private PIDController middlePID;
	private PIDController endPID;

	// Measures how close joints/arm are to being on target
	public enum AngleStates {
		DONE,
		CLOSE,
		CONT
	};

	// Angles for all positions
	public static double[] REST_ANGLES     = {0.825, 2.8, -2.9};
	public static double[] MID_CONE_ANGLES = {1.135, 1.86, -0.32};
	public static double[] MID_CUBE_ANGLES = {0.825, 1.826, 0.45};
	public static double[] TOP_CONE_ANGLES = {2.10, 0.3, 0.1}; // Old base - 2.15, 3/4 and before
	public static double[] TOP_CUBE_ANGLES = {2.1, 0.4, 0.0};
	public static double[] SHELF_ANGLES    = {0.825, 1.28, 1.31};
	public static double[] CHUTE_ANGLES    = {0.767, 2.556, -1.288};

	// Constructor
    public Arm() {
        baseMotor   = new CANSparkMax(5, MotorType.kBrushless);
        middleMotor = new CANSparkMax(7, MotorType.kBrushless);
        endMotor    = new CANSparkMax(6, MotorType.kBrushless);

		claw = new DoubleSolenoid(PNEU_CONTROLLER_ID, PneumaticsModuleType.REVPH, 15, 0);

        baseMotor.setIdleMode(IdleMode.kBrake);
        middleMotor.setIdleMode(IdleMode.kBrake);
        endMotor.setIdleMode(IdleMode.kBrake);

		baseMotor.setSmartCurrentLimit(60);
		middleMotor.setSmartCurrentLimit(60);
		endMotor.setSmartCurrentLimit(60);

		baseMotor.setInverted(true);
		middleMotor.setInverted(false);
		endMotor.setInverted(true);

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
		endPID    = new PIDController(p3, i3, 0);

		basePID.setTolerance(BASE_TOLERANCE);
		middlePID.setTolerance(MIDDLE_TOLERANCE);
		endPID.setTolerance(END_TOLERANCE);

		limitButton0 = new DigitalInput(0);
		limitButton9 = new DigitalInput(9);
    }

	public void hold(int joint) {
		if (joint == 1) {
			double torque = torqueJoint1(getActualBaseAngle(), getActualMiddleAngle(), getActualEndAngle());
			baseMotor.set(torque * BASE_TORQUE_TO_POWER);
		}
		else if (joint == 2) {
			double torque = torqueJoint2(getActualBaseAngle(), getActualMiddleAngle(), getActualEndAngle());
			middleMotor.set(torque * MIDDLE_TORQUE_TO_POWER);
		}
		else if (joint == 3) {
			double torque = torqueJoint3(getActualBaseAngle(), getActualMiddleAngle(), getActualEndAngle());
			endMotor.set(torque * END_TORQUE_TO_POWER);
		}
	}

	public void stopArm() {
		baseMotor.set(0);
		middleMotor.set(0);
		endMotor.set(0);
	}

	/**
	 * Moves joint to a target angle.
	 * PID powers are currently multiplied by rotational inertia.
	 * Optional parameter for speed multiplier.
	 * @param joint
	 * @param radians
	 * @return
	 */
	public AngleStates jointToAngle(int joint, double radians) {
		return jointToAngle(joint, radians, 1);
	}

	public AngleStates jointToAngle(int joint, double radians, double speedMult) {
		if (joint < 1 || joint > 3) {
			System.out.println("Joint should be 1-3");
			return AngleStates.DONE;
		}

		double q1 = getActualBaseAngle();
		double q2 = getActualMiddleAngle();
		double q3 = getActualEndAngle();

		if (joint == 1) {
			// Clamping angle to set limits
			radians = MathUtil.clamp(radians, ANGLE_1_MIN, ANGLE_1_MAX);

			// Movement with torque and PID
			double torquePower = torqueJoint1(q1, q2, q3) * BASE_TORQUE_TO_POWER;
			double pidPower    = basePID.calculate(q1, radians) * rotationalInertiaJoint1(q1, q2, q3) * speedMult;
			baseMotor.set(torquePower + pidPower);

			basePID.setSetpoint(radians);
			// Checking tolerances for DONE
			basePID.setTolerance(BASE_TOLERANCE);
			if (basePID.atSetpoint()) {
				return AngleStates.DONE;
			}

			// Checking tolerances for CLOSE
			basePID.setTolerance(BASE_TOLERANCE * 3);
			if (basePID.atSetpoint()) {
				return AngleStates.CLOSE;
			}

			return AngleStates.CONT;
		}
		else if (joint == 2) {
			// Clamping angle to set limits
			radians = MathUtil.clamp(radians, ANGLE_2_MIN, ANGLE_2_MAX);

			// Movement with torque and PID
			double torquePower = torqueJoint2(q1, q2, q3) * MIDDLE_TORQUE_TO_POWER;
			double pidPower    = middlePID.calculate(q2, radians) * rotationalInertiaJoint2(q1, q2, q3) * speedMult;
			middleMotor.set(torquePower + pidPower);

			// Checking tolerances for DONE
			middlePID.setTolerance(MIDDLE_TOLERANCE);
			if (middlePID.atSetpoint()) {
				return AngleStates.DONE;
			}

			// Checking tolerances for CLOSE
			middlePID.setTolerance(MIDDLE_TOLERANCE * 3);
			if (middlePID.atSetpoint()) {
				return AngleStates.CLOSE;
			}

			return AngleStates.CONT;
		}
		else if (joint == 3) {
			// Clamping angle to set limits
			radians = MathUtil.clamp(radians, ANGLE_3_MIN, ANGLE_3_MAX);

			// Movement with torque and PID
			double torquePower = torqueJoint3(q1, q2, q3) * END_TORQUE_TO_POWER;
			double pidPower    = endPID.calculate(q3, radians) * rotationalInertiaJoint3(q1, q2, q3) * speedMult;
			double totalPower  = MathUtil.clamp(torquePower + pidPower, -MAX_END_POWER, MAX_END_POWER);
			endMotor.set(totalPower);

			// Checking tolerances for DONE
			endPID.setTolerance(END_TOLERANCE);
			if (endPID.atSetpoint()) {
				return AngleStates.DONE;
			}

			// Checking tolerances for CLOSE
			endPID.setTolerance(END_TOLERANCE * 6);
			if (endPID.atSetpoint()) {
				return AngleStates.CLOSE;
			}

			return AngleStates.CONT;
		}

		// Should not get here
		return AngleStates.DONE;
	}

	public void powerEnd(double power) {
		double[] restArmPowers = restArmPowers();

		// If arm is folded in, base and middle don't need powers to hold their position
		if (Controls.armState == ArmStates.REST || Controls.armState == ArmStates.GRAB) {
			setMiddlePower(0);
			setEndPower(0);
		}
		else {
			hold(1);
			hold(2);
		}
		setEndPower(power + restArmPowers[3]);
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
	// Maxes out ~5.5 with a cone at full reach
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
		
		// Intertia of point is mr^2
		double joint2Inertia = JOINT_2_MASS * Math.pow(joint2R, 2);
		double joint3Inertia = JOINT_3_MASS * Math.pow(joint3R, 2);
		double clawInertia = 0;
		if (Controls.currentObject == Objects.CUBE) {
			clawInertia  = CUBE_MASS * Math.pow(endR, 2); 
		}
		else if (Controls.currentObject == Objects.CONE) {
			clawInertia  = CONE_MASS * Math.pow(endR, 2);
		}

		// Intertia of line uses integral
		double baseInertia   = BASE_MASS   * 1/3 * (Math.pow(joint2R, 3) - 0);
		double middleInertia = MIDDLE_MASS * 1/3 * (Math.pow(joint3R, 3) - Math.pow(joint2R, 3));
		double endInertia    = END_MASS    * 1/3 * (Math.pow(endR, 3)    - Math.pow(joint3R, 3));
		
		return joint2Inertia + joint3Inertia + clawInertia + baseInertia + middleInertia + endInertia;
	}

	// Maxes out ~1.8 with a cone at full reach
	public double rotationalInertiaJoint2(double q1, double q2, double q3) {
		// All distances are relative to Joint 2
		double joint3X       = LENGTH_MIDDLE * Math.cos(q1 + q2);
		double joint3Y       = LENGTH_MIDDLE * Math.sin(q1 + q2);
		double joint3R       = LENGTH_MIDDLE;
		
		double endX          = joint3X + LENGTH_END * Math.cos(q1 + q2 + q3);
		double endY          = joint3Y + LENGTH_END * Math.sin(q1 + q2 + q3);	
		double endR          = Math.hypot(endX, endY);
		
		// Intertia of point is mr^2
		double joint3Inertia = JOINT_3_MASS * Math.pow(joint3R, 2);
		double clawInertia = 0;
		if (Controls.currentObject == Objects.CUBE) {
			clawInertia  = CUBE_MASS * Math.pow(endR, 2); 
		}
		else if (Controls.currentObject == Objects.CONE) {
			clawInertia  = CONE_MASS * Math.pow(endR, 2);
		}

		// Inertia of line uses integral
		double middleInertia = MIDDLE_MASS * 1/3 * (Math.pow(joint3R, 3) - 0);
		double endInertia    = END_MASS    * 1/3 * (Math.pow(endR, 3)    - Math.pow(joint3R, 3));
		
		return joint3Inertia + clawInertia + middleInertia + endInertia;
	}

	// 0.196 when empty, 0.232 when holding cube, 0.526 when holding cone
	public double rotationalInertiaJoint3(double q1, double q2, double q3) {
		// All distances are relative to Joint 3
		double endR          = LENGTH_END;
		
		// Intertia of point is mr^2
		double clawInertia = 0;
		if (Controls.currentObject == Objects.CUBE) {
			clawInertia  = CUBE_MASS * Math.pow(endR, 2); 
		}
		else if (Controls.currentObject == Objects.CONE) {
			clawInertia  = CONE_MASS * Math.pow(endR, 2);
		}

		// Inertia of line uses integral
		double endInertia    = END_MASS    * 1/3 * (Math.pow(endR, 3) - 0);
		
		return clawInertia + endInertia;
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
	 * Methods to get angles of arms where 0 is straight back
	 * For physics calculations
	 */
	public double getActualBaseAngle() {
        return MathUtil.angleModulus(baseAbsoluteEncoder.getPosition() + 0.810);
    }

    public double getActualMiddleAngle() {
        return MathUtil.angleModulus(middleAbsoluteEncoder.getPosition() + 2.881);
    }

    public double getActualEndAngle() {
        return MathUtil.angleModulus(-1 * endAbsoluteEncoder.getPosition() + -3.010);
    }

	/*
	 * Pneumatics control
	 */
	public void openClaw() {
		// Some routines automatically open the claw. This ensures that controls tracks the claw being open.
		Controls.currentObject = Controls.Objects.EMPTY;
		claw.set(Value.kReverse);
	}

	public void closeClaw() {
		claw.set(Value.kForward);
	}

	/*
	 * Button methods
	 */
	public boolean limitButtonPressed() {
		boolean switchVal = (!limitButton0.get()) || (!limitButton9.get());
		if (switchVal == false) {
			switchCount++;
		}

		if ((switchCount > 25) && (switchVal == true)) {
			switchCount = 0;
			return true;
		}

		if (switchVal == true) {
			switchCount = 0;
		}

		return false;
	}

	/*
	 * Test functions
	 */
	public void testAbsEncoders() {
		if (printCount % 15 == 0) {
			System.out.println("Base:" + getActualBaseAngle() + " Middle:" + getActualMiddleAngle() + " End:" + getActualEndAngle());
		}
		printCount++;
	}

	public void testTorque() {
		double q1 = getActualBaseAngle();
		double q2 = getActualMiddleAngle();
		double q3 = getActualEndAngle();

		if (printCount % 15 == 0) {
			System.out.println("Base:" + torqueJoint1(q1, q2, q3) + " Total angle:" + (q1+q2+q3));
			//System.out.println(q1 + ", " + q2 + ", " + q3);
		}
		printCount++;
	}

	public void testRotInertia() {
		double q1 = getActualBaseAngle();
		double q2 = getActualMiddleAngle();
		double q3 = getActualEndAngle();

		if (printCount % 15 == 0) {
			System.out.println("End:" + rotationalInertiaJoint3(q1, q2, q3));
		}
		printCount++;

		// Trying to go to angle of 0
		double torquePower = torqueJoint1(q1, q2, q3) * BASE_TORQUE_TO_POWER;
		double pidPower    = basePID.calculate(q1, 1) * rotationalInertiaJoint1(q1, q2, q3);
		baseMotor.set(torquePower + pidPower);
		middleMotor.set(torqueJoint2(q1, q2, q3) * MIDDLE_TORQUE_TO_POWER);
		endMotor.set(torqueJoint3(q1, q2, q3) * END_TORQUE_TO_POWER);
	}


	public void testHoldPosition() {
		double q1 = getActualBaseAngle();
		double q2 = getActualMiddleAngle();
		double q3 = getActualEndAngle();

		double endTorque = torqueJoint3(q1, q2, q3);
		endMotor.set(endTorque * -0.035);

		double middleTorque = torqueJoint2(q1, q2, q3);
		middleMotor.set(middleTorque * 0.015);

		double baseTorque = torqueJoint1(q1, q2, q3);
		baseMotor.set(baseTorque * -0.01);
	}
}
