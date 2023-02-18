package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;

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

	//private DoubleSolenoid claw;

	// Last valid set of angles, in case user passes boundaries
	private double prevBaseAngle   = 0;
	private double prevMiddleAngle = 0;
	private double prevEndAngle    = 0;

	private double printCount = 0;

    // Constants - need to verify joint limits
    private final double LENGTH_BASE   = 0.5588;
    private final double LENGTH_MIDDLE = 0.53848;
    private final double LENGTH_END    = 0.4318;
	private final double ROBOT_FRONT   = -0.2032;
	private final double ROBOT_BACK    = 0.6096;
	
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
	private final double p1 = 0.12;
	private final double p2 = 0.08;
	private final double p3 = 0.16;

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

		//claw = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 0, 1);

        baseMotor.setIdleMode(IdleMode.kBrake);
        middleMotor.setIdleMode(IdleMode.kBrake);
        endMotor.setIdleMode(IdleMode.kBrake);

		baseMotor.setInverted(true);
		middleMotor.setInverted(true);

        baseAbsoluteEncoder   = baseMotor.getAbsoluteEncoder(Type.kDutyCycle);
        middleAbsoluteEncoder = middleMotor.getAbsoluteEncoder(Type.kDutyCycle);
        endAbsoluteEncoder    = endMotor.getAbsoluteEncoder(Type.kDutyCycle);

        baseAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        middleAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI);
        endAbsoluteEncoder.setPositionConversionFactor(2 * Math.PI);
		endAbsoluteEncoder.setInverted(true);
		baseAbsoluteEncoder.setZeroOffset(Math.PI/2 + 0.0332);
		endAbsoluteEncoder.setZeroOffset(5.432);
		
		prevBaseAngle   = baseAbsoluteEncoder.getPosition();
		prevMiddleAngle = middleAbsoluteEncoder.getPosition();
		prevEndAngle    = endAbsoluteEncoder.getPosition();

		basePID   = new PIDController(p1, 0, 0);
		middlePID = new PIDController(p2, 0, 0);
		endPID    = new PIDController(p3, 0, 0);

		basePID.setTolerance(BASE_TOLERANCE);
		middlePID.setTolerance(MIDDLE_TOLERANCE);
		endPID.setTolerance(END_TOLERANCE);
    }

	public int moveArmTo(double reachAngle, double reachDistance, double wristAngle) {
		// Target joint angles
		double[] targetJointAngles  = getJointAngles(reachAngle, reachDistance, wristAngle);
		for (int i = 0; i < 3; i++) {
			targetJointAngles[i] = MathUtil.angleModulus(targetJointAngles[i]);
		}
		
		basePID.setSetpoint(targetJointAngles[0]);
		middlePID.setSetpoint(targetJointAngles[1]);
		endPID.setSetpoint(targetJointAngles[2]);

		// Using current joint angles to get PID powers
		double[] currentJointAngles = {
			MathUtil.angleModulus(getBaseRotation()),
			MathUtil.angleModulus(getMiddleRotation()),
			MathUtil.angleModulus(getEndRotation())
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

    // Determines the angle of each joint. Wrist angle is relative to the floor in radians
    public double[] getJointAngles(double reachAngle, double reachDistance, double wristAngle) {
        // Total X and Y from base to end of claw
        double reachX = reachDistance * Math.cos(reachAngle);
        double reachY = reachDistance * Math.sin(reachAngle);

        // X and Y from base to joint 3
        double joint3X   = reachX - (LENGTH_END * Math.cos(wristAngle));
        double joint3Y   = reachY - (LENGTH_END * Math.sin(wristAngle));
        double joint3Hyp = Math.hypot(joint3X, joint3Y);
		
		// Avoid divide by 0 error
		if (joint3Hyp == 0) {
			joint3Hyp = 0.0001;
		}
		
		double joint3Angle = Math.atan2(joint3Y, joint3X);

        // Can now consider triangle between Joint 1, 2, and 3
		double height = triangleHeight(joint3Hyp, LENGTH_BASE, LENGTH_MIDDLE);
		
		// If arm is facing forward/down, a positive height goes down. We want height to be positive to avoid robot
		if (joint3X < 0) {
			height *= -1;
		}

        // Using value of height, we have 2 right triangles that can be solved for angle 1 and 2
        double angleFromJoint3ToArm1 = Math.asin(height / LENGTH_BASE);
		double q1 = joint3Angle + angleFromJoint3ToArm1;
		
		double q2 = -1 * (Math.PI - (Math.PI/2 - angleFromJoint3ToArm1) - Math.acos(height / LENGTH_MIDDLE));

		// Clamp q1 and q2 before calculating q3 so the wrist angle calculation will be based on the actual q1 and q2
		q1 = MathUtil.clamp(q1, ANGLE_1_MIN, ANGLE_1_MAX);
		q2 = MathUtil.clamp(q2, ANGLE_2_MIN, ANGLE_2_MAX);

		double q3 = wristAngle - (q1 + q2); // Rearrangement of q1 + q2 + q3 = wristAngle
		q3 = MathUtil.clamp(q3, ANGLE_3_MIN, ANGLE_3_MAX);
		
		// Determine if any part of arm intersects with robot
		// For all 3 segments, find when y = -0.05 meters below joint 1. If this occurs in the robot's x-values, reject the calculated angles and use the previous ones		
		double joint2X = LENGTH_BASE * Math.cos(q1);
		double joint2Y = LENGTH_BASE * Math.sin(q1);
		double baseIntersect = findRobotIntersection(0, 0, joint2X, q1);
		double middleIntersect = findRobotIntersection(joint2X, joint2Y, joint3X, (q1 + q2));
		
		double endIntersect = findRobotIntersection(joint3X, joint3Y, reachX, (q1 + q2 + q3));
		
		if (	(ROBOT_FRONT < baseIntersect   && baseIntersect   < ROBOT_BACK) ||
				(ROBOT_FRONT < middleIntersect && middleIntersect < ROBOT_BACK) ||
				(ROBOT_FRONT < endIntersect    && endIntersect    < ROBOT_BACK)) {
					q1 = prevBaseAngle;
					q2 = prevMiddleAngle;
					q3 = prevEndAngle;
				}
		
		prevBaseAngle   = q1;
		prevMiddleAngle = q2;
		prevEndAngle    = q3;
		
		double[] results = {q1, q2, q3};
		return results;
    }
	
	// Helper methods for getJointAngles()
	private double triangleHeight(double baseLength, double leg1Length, double leg2Length) {
		// Using Heron's formula, can find area of triangle
        double s = (baseLength + leg1Length + leg2Length) / 2;
        double area = Math.sqrt(s * (s - baseLength) * (s - leg1Length) * (s - leg2Length));

        // Using area, can find height from base
        double height = 2 * area / baseLength;
		return height;
	}
	
	private double findRobotIntersection(double startX, double startY, double endX, double radians) {
		// Standard calculation for intersection of 2 lines --> ax + b = -0.05
		if (radians % (Math.PI) == Math.PI / 2) {
			radians += 0.001;
		}
			
		double slope = Math.tan(radians);

		double yIntercept = (0 - startX) * slope + startY;
		
		double robotInterceptX = (-1 * yIntercept - 0.05) / slope;

		// If slope is 0, or Y-intercept is outside of the start and end X values, the robot is not in danger
		if (robotInterceptX < Math.min(startX, endX) || robotInterceptX > Math.max(startX, endX)) {
			return -100;
		}
		if (slope == 0) {
			return -100;
		}

		return robotInterceptX;
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
		double clawTorque    = 0            * -1 * endX; // Implement cone/cube mass
		
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
		double clawTorque    = 0            * -1 * endX; // Implement cone/cube mass
		
		double middleTorque  = MIDDLE_MASS  * -1 * middleCenterX;
		double endTorque     = END_MASS     * -1 * endCenterX;
		
		return joint3Torque + clawTorque + middleTorque + endTorque;
	}
	
	public double torqueJoint3(double q1, double q2, double q3) {
		// All distances are relative to Joint 3
		double endX          = LENGTH_END * Math.cos(q1 + q2 + q3);
		double endCenterX    = endX / 2;
		
		double clawTorque    = 0        * -1 * endX; // Implement cone/cube mass
		double endTorque     = END_MASS * -1 * endCenterX;
		
		return clawTorque + endTorque;
	}
	
	// kg * m^2
	// Only necessary if we focus on acceleration (probably not)
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

    public double getBaseRotation() {
        return baseAbsoluteEncoder.getPosition();
    }

    public double getMiddleRotation() {
        return middleAbsoluteEncoder.getPosition();
    }

    public double getEndRotation() {
        return endAbsoluteEncoder.getPosition();
    }

	/*
	public void openClaw() {
		claw.set(Value.kReverse);
	}

	public void closeClaw() {
		claw.set(Value.kForward);
	}*/

	// Test functions
	public void testAbsEncoders() {
		if (printCount % 15 == 0) {
			System.out.println("Base:" + baseAbsoluteEncoder.getPosition() + " Middle:" + middleAbsoluteEncoder.getPosition() + " End:" + endAbsoluteEncoder.getPosition());
		}
		printCount++;
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
