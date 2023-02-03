package frc.robot;

public class Auto {
    private boolean firstTimeDrive = true;
    private int step;
    private Drive drive;



    public Auto(Drive drive) {
        this.drive = drive;
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
