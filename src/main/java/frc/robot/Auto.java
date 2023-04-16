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
    private int step;
    private boolean firstTime = true;

    private int armStep;
    private boolean armFirstTime = true;

    private long delayEnd = 0; // Stores when delay() should return Robot.DONE
    private boolean delayFirstTime = true;

    // Object Creation
    private Drive          drive;
    private PoseEstimation position;
    private Arm            arm;
    private CustomTables   nTables;
    private Controls       controls;

    // Constants for starting poses for each auto
    public final Translation2d RAMP_RED_START    = new Translation2d(1.767, 4.699);
    public final Translation2d RAMP_BLUE_START   = new Translation2d(1.767, 3.302);
    public final Translation2d WALL_RED_START    = new Translation2d(1.767, 6.3754);
    public final Translation2d WALL_BLUE_START   = new Translation2d(1.767, 1.6256);
    // For now, center auto mirrors wall auto. With April Tags, these will change
    public final Translation2d CENTER_RED_START  = new Translation2d(1.767, 1.6256);
    public final Translation2d CENTER_BLUE_START = new Translation2d(1.767, 6.3754);

    // Variables
    private Pose2d[] rampAutoExitCommunity = new Pose2d[1]; // Stores location that will ensure we leave community
    private Pose2d[] autoSecondPieceRotate = new Pose2d[1];
    private double balancedRoll = 0;

    // Coordinates to be used in routines
    private static final double[][] autoCoordinates = {
        {2.0, 0.0, Math.PI/2}
    };
    private Pose2d[] listOfPoints = new Pose2d[autoCoordinates.length];

    // Constructor
    public Auto(Drive drive, PoseEstimation position, Arm arm, Controls controls) {
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
        this.controls = controls;
        this.nTables  = CustomTables.getInstance();
    }

    /**
     * 
     * @param isRed
     * @param delaySeconds
     * @return status
     */
    public int wallAuto(boolean isRed, long delaySeconds) {
        int    status   = Robot.CONT;
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

                if (isRed == true) {
                    drive.rotateWheels(0.833, 1.0256, 0);
                }
                else {
                    drive.rotateWheels(0.833, -1.0256, 0);
                }                
                break;
            case 2:
                // Place object we're holding
                status = armToTopCone();
                Robot.acceptedArmState = ArmStates.TOP_CONE;

                if (isRed == true) {
                    drive.rotateWheels(0.833, 1.0256, 0);
                }
                else {
                    drive.rotateWheels(0.833, -1.0256, 0);
                }	
                break;
            case 3:
                arm.openClaw();
                Controls.currentObject = Objects.EMPTY;
                status = Robot.DONE;
                break;
            case 4:
                AngleStates armStatus = armToRestPosition(true);
                Robot.acceptedArmState = ArmStates.REST;
                if (armStatus == AngleStates.DONE || armStatus == AngleStates.CLOSE) {
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
                    pose3 = new Pose2d(6.3, 7.101, new Rotation2d(0));
                }
                else {
                    pose1 = new Pose2d(2.6, 0.8, new Rotation2d(Math.PI));
                    pose2 = new Pose2d(2.6, 0.8, new Rotation2d(0));
                    pose3 = new Pose2d(6.3, 1.0, new Rotation2d(0));
                }

                status = drive.autoDriveToPoints(new Pose2d[]{pose2, pose3}, currPose);
                resetArmRoutines();
                break;
            case 6:
                // Bring down wrist
                status = armToGrabPosition();
                Robot.acceptedArmState = ArmStates.GRAB;
                Controls.armState = ArmStates.GRAB;
                break;
            case 7:
                // Align with cone
                double angleError = nTables.getGamePieceX();
                status = drive.alignWithPiece(angleError, false);
                break;
            case 8:
                // Drive to cone
                status = drive.driveToCone(1.0, controls.getLimitSwitch(), currPose.getTranslation());
                break;
            case 9:
                // Pick up cone
                arm.closeClaw();
                Controls.currentObject = Objects.CONE;
                status = Robot.DONE;
                break;
            case 10:
                // Retract arm
                AngleStates armStatus2 = armToRestPosition(false);
                Robot.acceptedArmState = ArmStates.REST;
                if (armStatus2 == AngleStates.DONE) {
                    status = Robot.DONE;
                }
                break;
            case 11:
                // Storing current location
                autoSecondPieceRotate[0] = new Pose2d(currPose.getX(), currPose.getY(), new Rotation2d(Math.PI));
                status = Robot.DONE;
                break;
            case 12:
                // Rotating
                status = drive.autoDriveToPoints(autoSecondPieceRotate, currPose);
                break;
            default:
                // Finished routine
                step = 1;
                firstTime = true;

                Controls.armState = ArmStates.REST;
                Robot.acceptedArmState = ArmStates.REST;

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
     * Places ojbect and balances on ramp - safe choice
     * @param isRed
     * @param objectPlaced
     * @param delaySeconds
     * @return status
     */
    public int rampAuto(boolean isRed, Objects objectPlaced, long delaySeconds) {
        int status = Robot.CONT;
    
		if (firstTime == true) {
			firstTime = false;
			step = 1;

            arm.closeClaw();
            Controls.currentObject = objectPlaced;
		}

        switch(step) {
            case 1:
                // Delay
                status = autoDelay(delaySeconds);
                drive.rotateWheels(-1, 0, 0);
                break;
            case 2:
                // Place object we're holding
                if (objectPlaced == Objects.CONE) {
                    status = armToTopCone();
                    Robot.acceptedArmState = ArmStates.TOP_CONE;
                }
                else {
                    status = armToTopCube();
                    Robot.acceptedArmState = ArmStates.TOP_CUBE;
                }
                
                drive.rotateWheels(-1, 0, 0);
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
     * Places cube or cone, leaves community, and balances on ramp
     * For cone placements, the robot shifts toward the middle of the ramp based on param shiftLeft
     * @param isRed
     * @param shiftLeft
     * @param delaySeconds
     * @return status
     */
    public int rampAutoFull(boolean isRed, boolean shiftLeft, long delaySeconds) {
        int    status   = Robot.CONT;
        Pose2d currPose = position.getOdometryPose();
    
		if (firstTime == true) {
			firstTime = false;
			step = 1;

            arm.closeClaw();
            Controls.currentObject = Objects.CONE;
		}

        switch(step) {
            case 1:
                // Delay
                status = autoDelay(delaySeconds);
                drive.rotateWheels(-1, 0, 0);         
                break;
            case 2:
                // Place object we're holding
                status = armToTopCone();
                Robot.acceptedArmState = ArmStates.TOP_CONE;
                
                drive.rotateWheels(-1, 0, 0);
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
                // Exit ramp with back side
                status = drive.leaveRamp(false);
                break;
            case 7:
                // Storing a pose 1.55 meter beyond ramp and straightened so we ensure we leave community
                if (shiftLeft) {
                    rampAutoExitCommunity[0] = new Pose2d(currPose.getX() + 1.55, currPose.getY() + 0.56, new Rotation2d(Math.PI));
                }
                else {
                    rampAutoExitCommunity[0] = new Pose2d(currPose.getX() + 1.55, currPose.getY() - 0.56, new Rotation2d(Math.PI));
                }
                status = Robot.DONE;
                break;
            case 8:
                // Exiting community
                status = drive.autoDriveToPoints(rampAutoExitCommunity, currPose);
                break;
            case 9:
                // Find drifted roll of ground - ramp should be the same angle
                balancedRoll = drive.getRoll();
                status = Robot.DONE;
                break;
            case 10:
                // Charge toward ramp with front side
                status = drive.chargeRamp(true);
                break;
            case 11:
                // Balance on ramp
                status = drive.balanceRamp(balancedRoll);
                break;
            case 12:
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
     * @param delaySeconds
     * @return status
     */
    public int centerAuto(boolean isRed, long delaySeconds) {
        int    status   = Robot.CONT;
        Pose2d currPose = position.getOdometryPose();
    
		if (firstTime == true) {
			firstTime = false;
			step = 1;
            System.out.println("Starting Center Auto");

            arm.closeClaw();
            Controls.currentObject = Objects.CONE;
		}

        switch(step) {
            case 1:
                // Delay
                status = autoDelay(delaySeconds);

                if (isRed == true) {
                    drive.rotateWheels(0.833, -1.0256, 0);
                }
                else {
                    drive.rotateWheels(0.833, 1.0256, 0);
                }                       
                break;
            case 2:
                // Place object we're holding
                status = armToTopCone();
                Robot.acceptedArmState = ArmStates.TOP_CONE;

                if (isRed == true) {
                    drive.rotateWheels(0.833, -1.0256, 0);
                }
                else {
                    drive.rotateWheels(0.833, 1.0256, 0);
                }
                break;
            case 3:
                arm.openClaw();
                Controls.currentObject = Objects.EMPTY;
                status = Robot.DONE;
                break;
            case 4:
                AngleStates armStatus = armToRestPosition(true);
                if (armStatus == AngleStates.DONE || armStatus == AngleStates.CLOSE) {
                    status = Robot.DONE;
                }
                Robot.acceptedArmState = ArmStates.REST;
                break;
            case 5:
                // Approach 1st object
                Pose2d pose1;
                Pose2d pose2;
                Pose2d pose3;

                if (isRed == false) {
                    pose1 = new Pose2d(2.6, 7.401, new Rotation2d(Math.PI));
                    pose2 = new Pose2d(2.6, 7.401, new Rotation2d(0));
                    pose3 = new Pose2d(6.3, 7.151, new Rotation2d(0));
                }
                else {
                    pose1 = new Pose2d(2.6, 0.8, new Rotation2d(Math.PI));
                    pose2 = new Pose2d(2.6, 0.8, new Rotation2d(0));
                    pose3 = new Pose2d(6.3, 1.05, new Rotation2d(0));
                }

                status = drive.autoDriveToPoints(new Pose2d[]{pose2, pose3}, currPose);
                resetArmRoutines();
                break;
            case 6:
                // Bring down wrist
                status = armToGrabPosition();
                Robot.acceptedArmState = ArmStates.GRAB;
                Controls.armState = ArmStates.GRAB;
                break;
            case 7:
                // Align with cone
                double angleError = nTables.getGamePieceX();
                status = drive.alignWithPiece(angleError, false);
                break;
            case 8:
                // Drive to cone
                status = drive.driveToCone(1.0, controls.getLimitSwitch(), currPose.getTranslation());
                break;
            case 9:
                // Pick up cone
                arm.closeClaw();
                Controls.currentObject = Objects.CONE;
                status = Robot.DONE;
                break;
            case 10:
                // Retract arm
                AngleStates armStatus2 = armToRestPosition(false);
                Robot.acceptedArmState = ArmStates.REST;
                if (armStatus2 == AngleStates.DONE) {
                    status = Robot.DONE;
                }
                break;
            case 11:
                // Storing current location
                autoSecondPieceRotate[0] = new Pose2d(currPose.getX(), currPose.getY(), new Rotation2d(Math.PI));
                status = Robot.DONE;
                break;
            case 12:
                // Rotating
                status = drive.autoDriveToPoints(autoSecondPieceRotate, currPose);
                break;
            default:
                // Finished routine
                step = 1;
                firstTime = true;

                Controls.armState = ArmStates.REST;
                Robot.acceptedArmState = ArmStates.REST;

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


    /****************************************************************************************** 
    *
    *    ARM FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * 
     * @param fromTop
     * @return
     */
    public AngleStates armToRestPosition(boolean fromTop) {    
		if (armFirstTime == true) {
			armFirstTime = false;
			armStep = 1;
		}

        if (fromTop) {
            switch(armStep) {
                case 1:
                    // Arm back - Base back some and wrist down to compensate
                    AngleStates status = arm.jointToAngle(1, 1.7, 1);
                    arm.jointToAngle(3, 0.6, 3);
                    arm.hold(2);
    
                    if (status == AngleStates.CLOSE || status == AngleStates.DONE) {
                        armStep++;
                    }
                    break;
                case 2:
                    // Wrist back some
                    AngleStates intermediateStatus = arm.jointToAngle(3, -Math.PI/2, 6);
                    arm.hold(2);
                    arm.hold(1);
                   
                    if (intermediateStatus == AngleStates.CLOSE || intermediateStatus == AngleStates.DONE) {
                        armStep++;
                    }
                    break;
                case 3:
                    // All of arm in
                    AngleStates baseStatus   = arm.jointToAngle(1, Arm.REST_ANGLES[0], 1);
                    AngleStates middleStatus = arm.jointToAngle(2, Arm.REST_ANGLES[1], 9);
                    AngleStates endStatus    = arm.jointToAngle(3, Arm.REST_ANGLES[2], 3);
    
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

    /**
     * 
     * @return
     */
    public int armToGrabPosition() {    
		AngleStates status = arm.jointToAngle(3, -0.4, 2.5);
        arm.jointToAngle(1, Arm.REST_ANGLES[0]);
        arm.jointToAngle(2, Arm.REST_ANGLES[1], 2);

        if (status == AngleStates.DONE) {
            arm.hold(1);
            arm.hold(2);
            arm.hold(3);
            return Robot.DONE;
        }
        return Robot.CONT;
    }

    public void armToChuteCone() {    
        arm.jointToAngle(1, Arm.CHUTE_CONE_ANGLES[0]);
        arm.jointToAngle(2, Arm.CHUTE_CONE_ANGLES[1], 12);
		arm.jointToAngle(3, Arm.CHUTE_CONE_ANGLES[2], 3);
    }

    public void armToChuteCube() {    
        arm.jointToAngle(1, Arm.CHUTE_CUBE_ANGLES[0]);
        arm.jointToAngle(2, Arm.CHUTE_CUBE_ANGLES[1], 12);
		arm.jointToAngle(3, Arm.CHUTE_CUBE_ANGLES[2], 3);
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

    public int armToMidCube() {    
        double[] armAngles = Arm.MID_CUBE_ANGLES;

        AngleStates baseState   = arm.jointToAngle(1, armAngles[0], 1);
        AngleStates middleState = arm.jointToAngle(2, armAngles[1], 2);
        AngleStates endState    = arm.jointToAngle(3, armAngles[2], 1);
        
        if (baseState   == AngleStates.DONE &&
            middleState == AngleStates.DONE &&
            endState    == AngleStates.DONE) {
                return Robot.DONE;
        }        
        return Robot.CONT;
    }

    /**
     * 
     * @return
     */
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
                arm.jointToAngle(1, 1.4, 2);
                arm.jointToAngle(3, -2.4);

                // If base and middle are close to or at target position, go to next step
                if ((middleStatus == AngleStates.DONE || middleStatus == AngleStates.CLOSE)) {
                    armStep++;
                }
                break;
            case 2:
                // Base and middle out
                AngleStates baseStatus2   = arm.jointToAngle(1, armAngles[0], 2);
                AngleStates middleStatus2 = arm.jointToAngle(2, armAngles[1], 3);
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
                AngleStates endStatusEnd    = arm.jointToAngle(3, armAngles[2], 1);
                if (baseStatusEnd   == AngleStates.DONE &&
                    middleStatusEnd == AngleStates.DONE &&
                    endStatusEnd    == AngleStates.DONE) {
                        armStep++;
                }
                break;
            case 4:
                // Finished routine
                arm.hold(1);
                arm.hold(2);
                arm.hold(3);
                //arm.jointToAngle(1, armAngles[0]);
                //arm.jointToAngle(2, armAngles[1]);
                //arm.jointToAngle(3, armAngles[2]);
                armFirstTime = true;
                armStep = 1;
                return Robot.DONE;
        }
        
        return Robot.CONT;
    }

    /**
     * 
     * @return
     */
    public int armToTopConeTeleop() {    
        double[] armAngles = Arm.TOP_CONE_ANGLES;

        AngleStates baseStatus   = arm.jointToAngle(1, armAngles[0], 1);
        AngleStates middleStatus = arm.jointToAngle(2, armAngles[1], 2);
        AngleStates endStatus    = arm.jointToAngle(3, -1.0, 1); //-1.0

        if (baseStatus   == AngleStates.DONE &&
            middleStatus == AngleStates.DONE &&
            endStatus    == AngleStates.DONE) {
                return Robot.DONE;
        }
        return Robot.CONT;

        /*
		if (armFirstTime == true) {
			armFirstTime = false;
			armStep = 1;
		}

        switch(armStep) {
            case 1:
                AngleStates baseStatus   = arm.jointToAngle(1, armAngles[0], 0.5);
                AngleStates middleStatus = arm.jointToAngle(2, armAngles[1], 1);
                AngleStates endStatus    = arm.jointToAngle(3, -1.0, 0.5);

                if (baseStatus   == AngleStates.DONE &&
                    middleStatus == AngleStates.DONE &&
                    endStatus    == AngleStates.DONE) {
                        armStep++;
                }

                break;
            /*case 1:
                // Middle out, base slightly
                AngleStates middleStatus = arm.jointToAngle(2, Math.PI/6, 3);
                arm.jointToAngle(1, 1.4, 6);
                arm.jointToAngle(3, -2.4);

                // If base and middle are close to or at target position, go to next step
                if ((middleStatus == AngleStates.DONE || middleStatus == AngleStates.CLOSE)) {
                    armStep++;
                }
                break;
            case 2:
                // Base and middle out
                AngleStates baseStatus2   = arm.jointToAngle(1, armAngles[0], 1.5);
                AngleStates middleStatus2 = arm.jointToAngle(2, armAngles[1], 6);
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
                AngleStates endStatusEnd    = arm.jointToAngle(3, -1.5, 2);
                if (baseStatusEnd   == AngleStates.DONE &&
                    middleStatusEnd == AngleStates.DONE &&
                    endStatusEnd    == AngleStates.DONE) {
                        armStep++;
                }
                break;
            case 4:
                // Finished routine
                arm.hold(1);
                arm.hold(2);
                arm.hold(3);
                //arm.jointToAngle(1, armAngles[0]);
                //arm.jointToAngle(2, armAngles[1]);
                //arm.jointToAngle(3, armAngles[2]);
                armFirstTime = true;
                armStep = 1;
                return Robot.DONE;
        }
        
        return Robot.CONT;*/
    }

    /**
     * 
     * @return
     */
    public int armToTopCube() {    
        double[] armAngles = Arm.TOP_CUBE_ANGLES;
        
        AngleStates baseStatus   = arm.jointToAngle(1, armAngles[0], 0.75); // 0.75 speed
        AngleStates middleStatus = arm.jointToAngle(2, armAngles[1], 1.6);
        AngleStates endStatus    = arm.jointToAngle(3, armAngles[2], 0.4);

        if ((baseStatus   == AngleStates.DONE || baseStatus   == AngleStates.CLOSE) &&
            (middleStatus == AngleStates.DONE || middleStatus == AngleStates.CLOSE) &&
            (endStatus    == AngleStates.DONE || endStatus   == AngleStates.CLOSE)) {
                return Robot.DONE;
        }
        return Robot.CONT;

        /*
		if (armFirstTime == true) {
			armFirstTime = false;
			armStep = 1;
		}

        switch(armStep) {
            case 1:
                // Base and middle out
                AngleStates baseStatus   = arm.jointToAngle(1, armAngles[0], 1);
                AngleStates middleStatus = arm.jointToAngle(2, armAngles[1], 1);

                // If base and middle are close to or at target position, go to next step
                if ((middleStatus == AngleStates.DONE || middleStatus == AngleStates.CLOSE) &&
                    (baseStatus == AngleStates.DONE || baseStatus == AngleStates.CLOSE)) {
                    armStep++;
                }
                break;
            case 2:
                // End last
                AngleStates baseStatus2   = arm.jointToAngle(1, armAngles[0], 1);
                AngleStates middleStatus2 = arm.jointToAngle(2, armAngles[1], 1);
                AngleStates endStatus     = arm.jointToAngle(3, armAngles[2], 1);

                // Everything is close, go to last step
                if ((baseStatus2   == AngleStates.DONE || baseStatus2   == AngleStates.CLOSE) &&
                    (middleStatus2 == AngleStates.DONE || middleStatus2 == AngleStates.CLOSE) &&
                    (endStatus     == AngleStates.DONE || endStatus     == AngleStates.CLOSE)) {
                    armStep++;
                }
                break;
            case 3:
                // Finished routine
                arm.jointToAngle(1, armAngles[0]);
                arm.jointToAngle(2, armAngles[1]);
                arm.jointToAngle(3, armAngles[2]);
                armFirstTime = true;
                armStep = 1;
                return Robot.DONE;
        }
        
        return Robot.CONT;*/
    }

    /**
     * 
     * @return
     */
    public int armToShelf() { 
        double[] armAngles = Arm.SHELF_ANGLES;

		if (armFirstTime == true) {
			armFirstTime = false;
			armStep = 1;
		}

        switch(armStep) {/*
            case 1:
                // Base and middle out
                AngleStates baseStatus   = arm.jointToAngle(1, armAngles[0]);
                AngleStates middleStatus = arm.jointToAngle(2, armAngles[1], 4);
                arm.hold(3);

                // If base and middle are close to or at target position, go to next step
                if ((baseStatus   == AngleStates.DONE || baseStatus   == AngleStates.CLOSE) &&
                    (middleStatus == AngleStates.DONE || middleStatus == AngleStates.CLOSE)) {
                        armStep++;
                }
                break;*/
            case 1:
                // Wrist out
                AngleStates baseStatusEnd   = arm.jointToAngle(1, armAngles[0]);
                AngleStates middleStatusEnd = arm.jointToAngle(2, armAngles[1], 3);
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
                arm.jointToAngle(2, armAngles[1], 3);
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
                status = drive.rotateWheels(-1, 0, 0);
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
}
// End of the Auto class
