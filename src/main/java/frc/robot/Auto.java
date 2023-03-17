// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Arm.AngleStates;
import frc.robot.Controls.ArmStates;
import frc.robot.Controls.Objects;

public class Auto {
    // State tracking variables - each variable can only be used in one function at any time
    // All top level routines use firstTime and step, all helper routines have their own variables
    private boolean firstTime = true;
    private int step;

    private boolean armFirstTime = true;
    private int armStep;

    private boolean delayFirstTime = true;
    private long delayEnd = 0; // Stores when delay() should return Robot.DONE

    // Object Creation
    private Drive drive;
    private PoseEstimation position;
    private Arm arm;

    // Constants for starting poses for each auto
    public final Translation2d RAMP_RED_START    = new Translation2d(1.767, 4.699);
    public final Translation2d RAMP_BLUE_START   = new Translation2d(1.767, 3.302);
    public final Translation2d WALL_RED_START    = new Translation2d(1.767, 6.3754);
    public final Translation2d WALL_BLUE_START   = new Translation2d(1.767, 1.6256);
    public final Translation2d CENTER_RED_START  = new Translation2d(0, 0);
    public final Translation2d CENTER_BLUE_START = new Translation2d(0, 0);

    // Variables
    private Pose2d[] rampAutoExitCommunity = new Pose2d[1]; // Stores location that will ensure we leave community
    private double balancedRoll = 0;

    // Coordinates to be used in routines
    private static final double[][] autoCoordinates = {
        {2.0, 0.0, Math.PI/2}
    };
    private Pose2d[] listOfPoints = new Pose2d[autoCoordinates.length];

    // Constructor
    public Auto(Drive drive, PoseEstimation position, Arm arm) {
        // Iterating through array of poses
        for (int i = 0; i < autoCoordinates.length; i++) {
            // Passing each item of the inner array as an argument for a Pose object
            listOfPoints[i] = new Pose2d(
                autoCoordinates[i][0], 
                autoCoordinates[i][1], 
                new Rotation2d(autoCoordinates[i][2]));
        }

        this.drive    = drive;
        this.position = position;
        this.arm      = arm;
    }

    /**
     * 
     * @param isRed
     * @param numObjects
     * @param delaySeconds
     * @return status
     */
    public int wallAuto(boolean isRed, int numObjects, long delaySeconds) {
        int status = Robot.CONT;
        Pose2d currPose = position.getOdometryPose();
    
		if (firstTime == true) {
			firstTime = false;
			step = 1;
            System.out.println("Starting Wall Auto");

            arm.closeClaw();
            Controls.currentObject = Objects.CONE;
		}

        switch(step) {
            case 1:
                // Delay
                status = autoDelay(delaySeconds);
                drive.rotateWheels(-1, 0, 0, false);
                break;
            case 2:
                // Place object we're holding
                status = armToTopCone();
                drive.rotateWheels(-1, 0, 0, false);
		Robot.acceptedArmState = ArmStates.TOP_CONE;
                break;
            case 3:
                arm.openClaw();
                Controls.currentObject = Objects.EMPTY;
                status = Robot.DONE;
                break;
            case 4:
                AngleStates armStatus = armToRestPosition(true);
                if (armStatus == AngleStates.DONE || armStatus == AngleStates.CLOSE) {
		    Robot.acceptedArmState = ArmStates.REST;
                    status = Robot.DONE;
                }
                break;
            case 5:
                // Approach 1st object
                Pose2d pose1;
                Pose2d pose2;
                Pose2d pose3;

                if (isRed == true) {
                    pose1 = new Pose2d(2.6, 7.401, new Rotation2d(Math.PI));
                    pose2 = new Pose2d(2.6, 7.401, new Rotation2d(0));
                    pose3 = new Pose2d(6.0, 7.101, new Rotation2d(0));
                }
                else {
                    pose1 = new Pose2d(2.6, 0.8, new Rotation2d(Math.PI));
                    pose2 = new Pose2d(2.6, 0.8, new Rotation2d(0));
                    pose3 = new Pose2d(6.0, 1.0, new Rotation2d(0));
                }

                status = drive.autoDriveToPoints(new Pose2d[]{pose1, pose2, pose3}, position.getOdometryPose());
                break;
            case 6:
                status = armToGrabPosition();
                break;/*
            case 5:
                // Bring down wrist
                status = autoDelay(2);
                break;
            case 6:
                // Close claw
                status = Robot.DONE;
                break;
            case 7:
                // Approach community
                if (isRed == true) {
                    pose1 = new Pose2d(3, 7.2, new Rotation2d(0));
                    pose2 = new Pose2d(3, 7.2, new Rotation2d(Math.PI));
                    pose3 = new Pose2d(WALL_RED_START, new Rotation2d(Math.PI));
                }
                else {
                    pose1 = new Pose2d(3, 0.6, new Rotation2d(0));
                    pose2 = new Pose2d(3, 0.6, new Rotation2d(Math.PI));
                    pose3 = new Pose2d(WALL_BLUE_START, new Rotation2d(Math.PI));
                }
                
                status = drive.autoDriveToPoints(new Pose2d[]{pose1, pose2, pose3}, position.getVisionPose());
                break;
            case 8:
                // Reach arm to top
                status = autoDelay(2);
                break;
            case 9:
                // Open claw
                status = Robot.DONE;
                break;*/
            default:
                // Finished routine
                step = 1;
                firstTime = true;

                Controls.armState = ArmStates.GRAB;
                Robot.acceptedArmState = ArmStates.GRAB;

                // Stops applicable motors
                drive.stopWheels();
                return Robot.DONE;
        }

        // If we are done with a step, we go on to the next one and continue the routine
        if (status == Robot.DONE) {
            step++;
        }
        
        return Robot.CONT;
    }

    /**
     * Places cone and balances on ramp - safe choice
     * @param isRed
     * @param numObjects
     * @param delaySeconds
     * @return status
     */
    public int rampAuto(boolean isRed, double numObjects, long delaySeconds) {
        int status = Robot.CONT;
    
		if (firstTime == true) {
			firstTime = false;
			step = 1;

            arm.closeClaw();
            Controls.currentObject = Objects.CONE;
		}

        // Vision pose not helpful for this auto
        Pose2d currPose = position.getOdometryPose();

        switch(step) {
            case 1:
                // Delay
                status = autoDelay(delaySeconds);
                drive.rotateWheels(-1, 0, 0, false);
                break;
            case 2:
                // Place object we're holding
                status = armToTopCone();
		Robot.acceptedArmState = ArmStates.TOP_CONE;
                drive.rotateWheels(-1, 0, 0, false);
                break;
            case 3:
                arm.openClaw();
                Controls.currentObject = Objects.EMPTY;
                status = Robot.DONE;
                break;
            case 4:
                AngleStates armStatus = armToRestPosition(true);
		Robot.acceptedArmState = ArmStates.REST;
                if (armStatus == AngleStates.CLOSE || armStatus == AngleStates.DONE) {
                    status = Robot.DONE;
                }
                balancedRoll = drive.getRoll();
                break;
            case 5:
                arm.stopArm();
                status = drive.chargeRamp(false);
                break;
            case 6:
                // Balance on ramp
                status = drive.balanceRamp(balancedRoll);
                break;
            case 7:
                // Lock wheels
                status = autoDelay(1);
                drive.crossWheels();
                break;
            default:
                // Finished routine
                step = 1;
                firstTime = true;

                // Stops applicable motors
                drive.stopWheels();
                return Robot.DONE;
        }

        //If we are done with a step, we go on to the next one and continue the routine
        if (status == Robot.DONE) {
            step++;
        }
        
        return Robot.CONT;
    }
	
	/**
     * Places cone, leaves community, and balances on ramp
     * @param isRed
     * @param numObjects
     * @param delaySeconds
     * @return status
     */
    public int rampAutoFull(boolean isRed, double numObjects, long delaySeconds) {
        int status = Robot.CONT;
    
		if (firstTime == true) {
			firstTime = false;
			step = 1;

            arm.closeClaw();
            Controls.currentObject = Objects.CONE;
		}

        // Vision pose not helpful for this auto
        Pose2d currPose = position.getOdometryPose();

        switch(step) {
            case 1:
                // Delay
                status = autoDelay(delaySeconds);
                drive.rotateWheels(-1, 0, 0, false);
                break;
            case 2:
                // Place object we're holding
                status = armToTopCone();
		Robot.acceptedArmState = ArmStates.TOP_CONE;
                drive.rotateWheels(-1, 0, 0, false);
                break;
            case 3:
                arm.openClaw();
                Controls.currentObject = Objects.EMPTY;
                status = Robot.DONE;
                break;
            case 4:
                AngleStates armStatus = armToRestPosition(true);
		Robot.acceptedArmState = ArmStates.REST;
                if (armStatus == AngleStates.CLOSE || armStatus == AngleStates.DONE) {
                    status = Robot.DONE;
                }
                balancedRoll = drive.getRoll();
                break;
            case 5:
                arm.stopArm();
                status = drive.chargeRamp(false);
                break;
            case 6:
                // Balance on ramp
                status = drive.balanceRamp(balancedRoll);
                break;
            case 7:
                // Exit ramp with back side
                status = drive.leaveRamp(false);
                break;
            case 8:
                // Storing a pose 1.25 meter beyond ramp and straightened so we ensure we leave community
                rampAutoExitCommunity[0] = new Pose2d(currPose.getX() + 1.25, currPose.getY(), new Rotation2d(Math.PI));
                status = Robot.DONE;
                break;
            case 9:
                // Exiting community
                status = drive.autoDriveToPoints(rampAutoExitCommunity, currPose);
                break;
            case 10:
                // Find drifted roll of ground - ramp should be the same angle
                balancedRoll = drive.getRoll();
                status = Robot.DONE;
                break;
            case 11:
                // Charge toward ramp with front side
                status = drive.chargeRamp(true);
                break;
            case 12:
                // Balance on ramp
                status = drive.balanceRamp(balancedRoll);
                break;*/
            case 13:
                // Lock wheels
                status = autoDelay(1);
                drive.crossWheels();
                break;
            default:
                // Finished routine
                step = 1;
                firstTime = true;

                // Stops applicable motors
                drive.stopWheels();
                return Robot.DONE;
        }

        //If we are done with a step, we go on to the next one and continue the routine
        if (status == Robot.DONE) {
            step++;
        }
        
        return Robot.CONT;
    }

    /**
     * 
     * @param isRed
     * @param numObjects
     * @param delaySeconds
     * @return status
     */
    public int centerAuto(boolean isRed, int numObjects, double delaySeconds) {
        return Robot.DONE;
    }

    /*
     * Arm functions
     */
    public AngleStates armToRestPosition(boolean fromTop) {    
		if (armFirstTime == true) {
			armFirstTime = false;
			armStep = 1;
		}

        if (fromTop) {
            switch(armStep) {
                case 1:
                    // Wrist in
                    AngleStates status = arm.jointToAngle(3, Arm.REST_ANGLES[2], 1.6);
                    arm.hold(1);
                    arm.hold(2);
    
                    if (status == AngleStates.CLOSE || status == AngleStates.DONE) {
                        armStep++;
                    }
                    break;
                case 2:
                    // Middle in
                    AngleStates intermediateStatus = arm.jointToAngle(2, Arm.REST_ANGLES[1], 6);
                   
                    if (intermediateStatus == AngleStates.CLOSE || intermediateStatus == AngleStates.DONE) {
                        armStep++;
                    }
                    break;
                case 3:
                    // All of arm in
                    AngleStates baseStatus   = arm.jointToAngle(1, Arm.REST_ANGLES[0], 5);
                    AngleStates middleStatus = arm.jointToAngle(2, Arm.REST_ANGLES[1], 6);
                    AngleStates endStatus    = arm.jointToAngle(3, Arm.REST_ANGLES[2], 0.75);
    
                    // If all joints are done, robot goes to resting step, then returns DONE
                    if (baseStatus   == AngleStates.DONE &&
                        middleStatus == AngleStates.DONE &&
                        endStatus    == AngleStates.DONE) {
                            armStep++;
                    }
                    // If all joints are close or done, robot stays on this step and returns CLOSE
                    else if ((baseStatus  == AngleStates.DONE || baseStatus == AngleStates.CLOSE) &&
                        (middleStatus == AngleStates.DONE || middleStatus == AngleStates.CLOSE) &&
                        (endStatus    == AngleStates.DONE || endStatus    == AngleStates.CLOSE)) {
                            arm.stopArm();
                            return AngleStates.CLOSE;
                    }
                    break;
                default:
                    // Finished routine
                    arm.stopArm();
                    Robot.fromTop = false;
                    armFirstTime = true;
                    armStep = 1; 
                    return AngleStates.DONE;
            }
        }
        else {
            switch(armStep) {
                case 1:
                    // Wrist in
                    AngleStates status = arm.jointToAngle(3, Arm.REST_ANGLES[2], 1.6);
                    arm.hold(1);
                    arm.hold(2);
    
                    if (status == AngleStates.CLOSE || status == AngleStates.DONE) {
                        armStep++;
                    }
                    break;
                case 2:
                    // All of arm in
                    AngleStates baseStatus   = arm.jointToAngle(1, Arm.REST_ANGLES[0], 3);
                    AngleStates middleStatus = arm.jointToAngle(2, Arm.REST_ANGLES[1], 6);
                    AngleStates endStatus    = arm.jointToAngle(3, Arm.REST_ANGLES[2], 0.75);
    
                    // If all joints are done, robot goes to resting step, then returns DONE
                    if (baseStatus   == AngleStates.DONE &&
                        middleStatus == AngleStates.DONE &&
                        endStatus    == AngleStates.DONE) {
                            armStep++;
                    }
                    // If all joints are close or done, robot stays on this step and returns CLOSE
                    else if ((baseStatus  == AngleStates.DONE || baseStatus == AngleStates.CLOSE) &&
                        (middleStatus == AngleStates.DONE || middleStatus == AngleStates.CLOSE) &&
                        (endStatus    == AngleStates.DONE || endStatus    == AngleStates.CLOSE)) {
                            return AngleStates.CLOSE;
                    }
                    break;
                default:
                    // Finished routine
                    arm.stopArm();
                    armFirstTime = true;
                    armStep = 1;
                    return AngleStates.DONE;
            }
        }
        
        return AngleStates.CONT;
    }

    public int armToGrabPosition() {    
		AngleStates status = arm.jointToAngle(3, -0.28, 2.5);
        arm.jointToAngle(1, Arm.REST_ANGLES[0]);
        arm.jointToAngle(2, Arm.REST_ANGLES[1], 2);

        if (status == AngleStates.DONE) {
            arm.stopArm();
            return Robot.DONE;
        }
        return Robot.CONT;
    }

    public void armToChute() {    
        arm.jointToAngle(1, Arm.CHUTE_ANGLES[0]);
        arm.jointToAngle(2, Arm.CHUTE_ANGLES[1], 2);
		arm.jointToAngle(3, Arm.CHUTE_ANGLES[2], 2);
    }

    public int armToMidPosition(double[] armAngles) {    
		if (armFirstTime == true) {
			armFirstTime = false;
			armStep = 1;
		}

        switch(armStep) {
            case 1:
                // Base and middle out
                AngleStates baseStatus   = arm.jointToAngle(1, armAngles[0], 6);
                AngleStates middleStatus = arm.jointToAngle(2, armAngles[1], 6);
                arm.hold(3);

                // If base and middle are close to or at target position, go to next step
                if ((baseStatus   == AngleStates.DONE || baseStatus   == AngleStates.CLOSE) &&
                    (middleStatus == AngleStates.DONE || middleStatus == AngleStates.CLOSE)) {
                        armStep++;
                }
                break;
            case 2:
                // Wrist out
                AngleStates baseStatusEnd   = arm.jointToAngle(1, armAngles[0], 2);
                AngleStates middleStatusEnd = arm.jointToAngle(2, armAngles[1], 2);
                AngleStates endStatusEnd    = arm.jointToAngle(3, armAngles[2]);
                if ((baseStatusEnd   == AngleStates.DONE || baseStatusEnd   == AngleStates.CLOSE) &&
                    (middleStatusEnd == AngleStates.DONE || middleStatusEnd == AngleStates.CLOSE) &&
                    (endStatusEnd    == AngleStates.DONE || endStatusEnd    == AngleStates.CLOSE)) {
                    armStep++;
                }
                break;
            default:
                // Finished routine
                arm.jointToAngle(1, armAngles[0]);
                arm.jointToAngle(2, armAngles[1]);
                arm.jointToAngle(3, armAngles[2]);
                armFirstTime = true;
                armStep = 1;
                return Robot.DONE;
        }
        
        return Robot.CONT;
    }

    public int armToTopCone() {    
        double[] armAngles = Arm.TOP_CONE_ANGLES;

		if (armFirstTime == true) {
			armFirstTime = false;
			armStep = 1;
		}

        switch(armStep) {
            case 1:
                // Middle out, base slightly
                AngleStates middleStatus = arm.jointToAngle(2, Math.PI/6, 2.5);
                arm.jointToAngle(1, 1.4, 3);
                arm.jointToAngle(3, -2.4);

                // If base and middle are close to or at target position, go to next step
                if ((middleStatus == AngleStates.DONE || middleStatus == AngleStates.CLOSE)) {
                    armStep++;
                }
                break;
            case 2:
                // Base and middle out
                AngleStates baseStatus2   = arm.jointToAngle(1, armAngles[0], 1.5);
                AngleStates middleStatus2 = arm.jointToAngle(2, armAngles[1], 2);
                arm.hold(3);

                // If base and middle are close to or at target position, go to next step
                if ((baseStatus2   == AngleStates.DONE || baseStatus2   == AngleStates.CLOSE) &&
                    (middleStatus2 == AngleStates.DONE || middleStatus2 == AngleStates.CLOSE)) {
                    armStep++;
                }
                break;
            case 3:
                // Wrist out
                AngleStates baseStatusEnd   = arm.jointToAngle(1, armAngles[0], 0.7);
                AngleStates middleStatusEnd = arm.jointToAngle(2, armAngles[1]);
                AngleStates endStatusEnd    = arm.jointToAngle(3, armAngles[2]);
                if (baseStatusEnd   == AngleStates.DONE &&
                    middleStatusEnd == AngleStates.DONE &&
                    endStatusEnd    == AngleStates.DONE) {
                        armStep++;
                }
                break;
            case 4:
                // Finished routine
                arm.jointToAngle(1, armAngles[0]);
                arm.jointToAngle(2, armAngles[1]);
                arm.jointToAngle(3, armAngles[2]);
                armFirstTime = true;
                armStep = 1;
                return Robot.DONE;
        }
        
        return Robot.CONT;
    }

    public int armToTopCube() {    
        double[] armAngles = Arm.TOP_CUBE_ANGLES;
        
		if (armFirstTime == true) {
			armFirstTime = false;
			armStep = 1;
		}

        switch(armStep) {
            case 1:
                // Middle out
                AngleStates middleStatus = arm.jointToAngle(2, Math.PI/6, 3);
                arm.jointToAngle(1, 1.25);
                arm.jointToAngle(3, -2.2);

                // If base and middle are close to or at target position, go to next step
                if ((middleStatus == AngleStates.DONE || middleStatus == AngleStates.CLOSE)) {
                    armStep++;
                }
                break;
            case 2:
                // Base and middle out
                AngleStates baseStatus2   = arm.jointToAngle(1, armAngles[0], 1.2);
                AngleStates middleStatus2 = arm.jointToAngle(2, armAngles[1], 2);
                arm.hold(3);

                // If base and middle are close to or at target position, go to next step
                if ((baseStatus2   == AngleStates.DONE || baseStatus2   == AngleStates.CLOSE) &&
                    (middleStatus2 == AngleStates.DONE || middleStatus2 == AngleStates.CLOSE)) {
                    armStep++;
                }
                break;
            case 3:
                // Wrist out
                AngleStates baseStatusEnd   = arm.jointToAngle(1, armAngles[0]);
                AngleStates middleStatusEnd = arm.jointToAngle(2, armAngles[1]);
                AngleStates endStatusEnd    = arm.jointToAngle(3, armAngles[2]);
                if (baseStatusEnd   == AngleStates.DONE &&
                    middleStatusEnd == AngleStates.DONE &&
                    endStatusEnd    == AngleStates.DONE) {
                        armStep++;
                }
                break;
            case 4:
                // Finished routine
                arm.jointToAngle(1, armAngles[0]);
                arm.jointToAngle(2, armAngles[1]);
                arm.jointToAngle(3, armAngles[2]);
                armFirstTime = true;
                armStep = 1;
                return Robot.DONE;
        }
        
        return Robot.CONT;
    }

    public int armToShelf() { 
        double[] armAngles = Arm.SHELF_ANGLES;

		if (armFirstTime == true) {
			armFirstTime = false;
			armStep = 1;
		}

        switch(armStep) {
            case 1:
                // Base and middle out
                AngleStates baseStatus   = arm.jointToAngle(1, armAngles[0]);
                AngleStates middleStatus = arm.jointToAngle(2, armAngles[1], 2);
                arm.hold(3);

                // If base and middle are close to or at target position, go to next step
                if ((baseStatus   == AngleStates.DONE || baseStatus   == AngleStates.CLOSE) &&
                    (middleStatus == AngleStates.DONE || middleStatus == AngleStates.CLOSE)) {
                        armStep++;
                }
                break;
            case 2:
                // Wrist out
                AngleStates baseStatusEnd   = arm.jointToAngle(1, armAngles[0]);
                AngleStates middleStatusEnd = arm.jointToAngle(2, armAngles[1]);
                AngleStates endStatusEnd    = arm.jointToAngle(3, armAngles[2]);
                if ((baseStatusEnd   == AngleStates.DONE || baseStatusEnd   == AngleStates.CLOSE) &&
                    (middleStatusEnd == AngleStates.DONE || middleStatusEnd == AngleStates.CLOSE) &&
                    (endStatusEnd    == AngleStates.DONE || endStatusEnd    == AngleStates.CLOSE)) {
                    armStep++;
                }
                break;
            default:
                // Finished routine - does not move claw
                arm.jointToAngle(1, armAngles[0]);
                arm.jointToAngle(2, armAngles[1]);
                arm.jointToAngle(3, armAngles[2]);
                armFirstTime = true;
                armStep = 1;
                return Robot.DONE;
        }
        
        return Robot.CONT;
    }


    /****************************************************************************************** 
    *
    *    HELPER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Delays the program for a set number of seconds.
     * 
     * @param seconds
     * @return status
     */
    public int autoDelay(long seconds) {
        long currentMS = System.currentTimeMillis();

        if (delayFirstTime) {
            delayEnd = currentMS + (seconds * 1000);
            delayFirstTime = false;
        }

        if (currentMS > delayEnd) {
            delayFirstTime = true;
            return Robot.DONE;
        }
        return Robot.CONT;
    }

    public void resetArmRoutines() {
        armFirstTime = true;
        armStep = 1;
        delayFirstTime = true;
    }


    /****************************************************************************************** 
    *
    *    TEST FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * 
     * @return status
     */
    public int testRamp() {
        int status = Robot.CONT;
    
		if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        switch(step) {
            case 1:
                // Angles wheels before driving
                status = drive.rotateWheels(-1, 0, 0, false);
                break;
            case 2:
                status = drive.chargeRamp(false);
                break;
            case 3:
                status = drive.balanceRamp(0);
                break;
            case 4:
                status = autoDelay(2);
                drive.crossWheels();
                break;
            default:
                // Finished routine
                step = 1;
                firstTime = true;

                // Stops applicable motors
                drive.stopWheels();
 
                return Robot.DONE;
        }

        //If we are done with a step, we go on to the next one and continue the routine
        if (status == Robot.DONE) {
            step++;
        }
        
        return Robot.CONT;
    }

    /**
     * 
     * @return status
     */
    public int driveToPointsTest() {
        int status = Robot.CONT;
    
		if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        switch(step) {
            case 1:
                status = drive.autoDriveToPoints(listOfPoints, position.getVisionPose());
                break;
            default:
                // Finished routine
                step = 1;
                firstTime = true;

                // Stops applicable motors
                drive.stopWheels();
 
                return Robot.DONE;
        }

        //If we are done with a step, we go on to the next one and continue the routine
        if (status == Robot.DONE) {
            step++;
        }
        
        return Robot.CONT;
    }
}
// End of the Auto class
