package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Auto {
    private boolean firstTimeDrive = true;
    private int step;
    private Drive drive;

    private static final double[][] autoCoordinates = {
        {1, 0.0, Math.PI/2},
        {0.0, 1.0, Math.PI}
        //{0, 0.5, -Math.PI}
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
