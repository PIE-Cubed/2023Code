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

    // Constants for starting poses for each auto
    private final Pose2d RAMP_START        = new Pose2d(0, 0, new Rotation2d(Math.PI));
    private final Pose2d WALL_RED_START    = new Pose2d(0, 0, new Rotation2d(0));
    private final Pose2d WALL_BLUE_START   = new Pose2d(0, 0, new Rotation2d(0));
    private final Pose2d CENTER_RED_START  = new Pose2d(0, 0, new Rotation2d(0));
    private final Pose2d CENTER_BLUE_START = new Pose2d(0, 0, new Rotation2d(0));

    // Variables
    private Pose2d[] rampAutoExitCommunity = new Pose2d[1]; // Stores location that will ensure we leave community
    private double balancedPitch = 0;

    // Coordinates to be used in routines
    private static final double[][] autoCoordinates = {
        {0.55, 0.0, 0.0}
    };
    private Pose2d[] listOfPoints = new Pose2d[autoCoordinates.length];

    // Constructor
    public Auto(Drive drive) {
        // Iterating through array of poses
        for (int i = 0; i < autoCoordinates.length; i++) {
            // Passing each item of the inner array as an argument for a Pose object
            listOfPoints[i] = new Pose2d(
                autoCoordinates[i][0], 
                autoCoordinates[i][1], 
                new Rotation2d(autoCoordinates[i][2]));
        }

        this.drive = drive;
    }

    public int wallAuto(boolean redSide, int numCones, double delaySeconds) {
        return Robot.DONE;
    }

    public int rampAuto(long delaySeconds) {
        int status = Robot.CONT;
    
		if (firstTime == true) {
			firstTime = false;
			step = 1;
            drive.resetOdometry(RAMP_START);
		}

        switch(step) {
            case 1:
                // Delay
                System.out.println(delaySeconds);
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
                rampAutoExitCommunity[0] = new Pose2d(drive.getX() + 1.5, drive.getY(), new Rotation2d(Math.PI));
                status = Robot.DONE;
                break;
            case 7:
                // Exiting community
                status = drive.autoDriveToPoints(rampAutoExitCommunity);
                break;
            case 8:
                // Charge toward ramp with front side
                status = drive.chargeRamp(true);
                // Find drifted pitch of ground - ramp should be the same angle
                balancedPitch = drive.getPitch();
                break;
            case 9:
                // Balance on ramp
                status = drive.balanceRamp(balancedPitch);
                break;
            case 10:
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
                status = drive.autoDriveToPoints(listOfPoints);
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

    public int driveAuto(double distanceFeet) {
        int status = Robot.CONT;
    
		if (firstTime == true) {
			firstTime = false;
			step = 1;
		}

        switch(step) {
            case 1:
                status = drive.autoCrabDrive(distanceFeet, 0.10, 0);
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
