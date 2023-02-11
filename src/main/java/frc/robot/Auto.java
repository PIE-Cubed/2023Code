package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Auto {
    // Instance Variables
    private boolean firstTimeDrive = true;
    private boolean delayFirstTime = true;
    private long delayEnd = 0;
    private int step;

    // Object Creation
    private Drive drive;

    private static final double[][] autoCoordinates = {
        {0.55, 0.0, 0.0}
        //{0.0, 0.25, 0.0},
        //{-0.25, 0.25, 0.0}
    };

    private Pose2d[] listOfPoints = new Pose2d[autoCoordinates.length];

    public Auto(Drive drive) {
        // Iterating through array of poses
        for (int i = 0; i < autoCoordinates.length; i++) {
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

    public int rampAuto(boolean redSide, int numCones, double delaySeconds) {
        return Robot.DONE;
    }

    public int centerAuto(boolean redSide, int numCones, double delaySeconds) {
        return Robot.DONE;
    }

    public int testRamp() {
        int status = Robot.CONT;
    
		if (firstTimeDrive == true) {
			firstTimeDrive = false;
			step = 1;
		}

        switch(step) {
            case 1:
                // Angles wheels before driving
                status = drive.rotateWheels(-1, 0, 0, false);
                break;
            case 2:
                status = drive.chargeRamp();
                break;
            case 3:
                status = drive.balanceRamp();
                break;
            case 4:
                status = autoDelay(2);
                drive.crossWheels();
                break;
            default:
                // Finished routine
                step = 1;
                firstTimeDrive = true;

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
    
		if (firstTimeDrive == true) {
			firstTimeDrive = false;
			step = 1;
		}

        switch(step) {
            case 1:
                status = drive.autoDriveToPoints(listOfPoints);
                break;
            default:
                // Finished routine
                step = 1;
                firstTimeDrive = true;

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

    public int autoDelay(long seconds) {
        long currentMS = System.currentTimeMillis();

        if (delayFirstTime) {
            delayEnd = currentMS + (seconds * 1000);
        }

        if (currentMS > delayEnd) {
            delayFirstTime = true;
            return Robot.DONE;
        }
        return Robot.CONT;
    }

    public int driveAuto(double distanceFeet) {
        int status = Robot.CONT;
    
		if (firstTimeDrive == true) {
			firstTimeDrive = false;
			step = 1;
		}

        switch(step) {
            case 1:
                status = drive.autoCrabDrive(distanceFeet, 0.10, 0);
                break;
            default:
                // Finished routine
                step = 1;
                firstTimeDrive = true;

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
