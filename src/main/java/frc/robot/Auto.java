package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Auto {
    // State tracking variables - each variable can only be used in one function at any time
    // All top level routines use firstTime and step, all helper routines have their own variables
    private boolean firstTime = true;
    private int step;

    private boolean delayFirstTime = true;
    private long delayEnd = 0; // Stores when delay() should return Robot.DONE

    // Object Creation
    private Drive drive;
    private PoseEstimation position;

    // Constants for starting poses for each auto
    public final Pose2d RAMP_RED_START    = new Pose2d(0, 0, new Rotation2d(Math.PI));
    public final Pose2d RAMP_BLUE_START   = new Pose2d(0, 0, new Rotation2d(Math.PI));
    public final Pose2d WALL_RED_START    = new Pose2d(2.0, 7.5936, new Rotation2d(Math.PI));
    public final Pose2d WALL_BLUE_START   = new Pose2d(2.0, 0.4064, new Rotation2d(Math.PI));
    public final Pose2d CENTER_RED_START  = new Pose2d(0, 0, new Rotation2d(0));
    public final Pose2d CENTER_BLUE_START = new Pose2d(0, 0, new Rotation2d(0));

    // Variables
    private Pose2d[] rampAutoExitCommunity = new Pose2d[1]; // Stores location that will ensure we leave community
    private double balancedPitch = 0;

    // Coordinates to be used in routines
    private static final double[][] autoCoordinates = {
        {0.55, 0.0, 0.0}
    };
    private Pose2d[] listOfPoints = new Pose2d[autoCoordinates.length];

    // Constructor
    public Auto(Drive drive, PoseEstimation position) {
        // Iterating through array of poses
        for (int i = 0; i < autoCoordinates.length; i++) {
            // Passing each item of the inner array as an argument for a Pose object
            listOfPoints[i] = new Pose2d(
                autoCoordinates[i][0], 
                autoCoordinates[i][1], 
                new Rotation2d(autoCoordinates[i][2]));
        }

        this.drive = drive;
        this.position = position;
    }

    public int wallAuto(boolean redSide, int numCones, long delaySeconds) {
        int status = Robot.CONT;
    
		if (firstTime == true) {
			firstTime = false;
			step = 1;
            System.out.println("Starting Wall Auto");
            drive.setAngleAdjustment(180);
		}

        switch(step) {
            case 1:
                // Delay
                status = autoDelay(delaySeconds);
                break;
            case 2:
                // Place object we're holding
                status = Robot.DONE;
                break;
            case 3:
                // Rotating wheels before driving
                status = drive.rotateWheels(-1, 0, 0, false);
                break;
            case 4:
                // Approach 1st object
                Pose2d pose1;
                Pose2d pose2;
                Pose2d pose3;

                if (redSide) {
                    pose1 = new Pose2d(3, 7.2, new Rotation2d(Math.PI));
                    pose2 = new Pose2d(3, 7.2, new Rotation2d(0));
                    pose3 = new Pose2d(6.3716, 7, new Rotation2d(0));
                }
                else {
                    pose1 = new Pose2d(3, 0.8, new Rotation2d(Math.PI));
                    pose2 = new Pose2d(3, 0.8, new Rotation2d(0));
                    pose3 = new Pose2d(6.3716, 1, new Rotation2d(0));
                }
                status = drive.autoDriveToPoints(new Pose2d[]{pose1, pose2, pose3}, position.getVisionPose());
                break;
                /*
            case 5:
                // Bring down wrist
                status = autoDelay(2);
                break;
            case 6:
                // Close claw
                status = Robot.DONE;
                break;
            case 7:
                // Bring object back
                Pose2d homePose;
                if (redSide) {
                    homePose = new Pose2d(6.2, 7, new Rotation2d(Math.PI));
                }
                else {
                    homePose = new Pose2d(6.2, 1, new Rotation2d(Math.PI));
                }
                status = drive.autoDriveToPoints(new Pose2d[]{homePose});
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

    public int rampAuto(long delaySeconds) {
        int status = Robot.CONT;
    
		if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        Pose2d currPose = position.getVisionPose();

        switch(step) {
            case 1:
                // Delay
                status = autoDelay(delaySeconds);
                break;
            case 2:
                // Place object we're holding
                status = Robot.DONE;
                break;
            case 3:
                // Rotating wheels before driving
                status = drive.rotateWheels(-1, 0, 0, false);
                break;
            case 4:
                // Charge toward ramp with back side
                status = drive.chargeRamp(false);
                break;
            case 5:
                // Exit ramp with back side
                status = drive.leaveRamp(false);
                break;
            case 6:
                // Storing a pose 1.5 meter beyond ramp and straightened so we ensure we leave community
                rampAutoExitCommunity[0] = new Pose2d(currPose.getX() + 1.5, currPose.getY(), new Rotation2d(Math.PI));
                status = Robot.DONE;
                break;
            case 7:
                // Exiting community
                status = drive.autoDriveToPoints(rampAutoExitCommunity, currPose);
                break;
            case 8:
                // Find drifted pitch of ground - ramp should be the same angle
                balancedPitch = drive.getPitch();
                status = Robot.DONE;
                break;
            case 9:
                // Charge toward ramp with front side
                status = drive.chargeRamp(true);
                break;
            case 10:
                // Balance on ramp
                status = drive.balanceRamp(balancedPitch);
                break;
            case 11:
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

    public int centerAuto(boolean redSide, int numCones, double delaySeconds) {
        return Robot.DONE;
    }

    /*
     * HELPER ROUTINES
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

    /*
     * TEST ROUTINES
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