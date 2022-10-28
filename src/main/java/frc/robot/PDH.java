package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;

public class PDH {
    
    private static PowerDistribution powerDistribution;
    

    private static final int FL_ROTATE_ID = 0;
    private static final int FL_DRIVE_ID  = 0;
    private static final int FR_ROTATE_ID = 0;
    private static final int FR_DRIVE_ID  = 1;
    private static final int BL_ROTATE_ID = 0;
    private static final int BL_DRIVE_ID  = 2;
    private static final int BR_ROTATE_ID = 0;
    private static final int BR_DRIVE_ID  = 3;
    private static final int MTEOR_ID = 0;


    public PDH () {
        powerDistribution = new PowerDistribution();
    }

    public static void checkFaultyDriveMotors(){
        double FLCurrent = powerDistribution.getCurrent(FL_DRIVE_ID);
        double FRCurrent = powerDistribution.getCurrent(FR_DRIVE_ID);
        double BLCurrent = powerDistribution.getCurrent(BL_DRIVE_ID);
        double BRCurrent = powerDistribution.getCurrent(BR_DRIVE_ID);

        double[] currentArray = {FLCurrent, FRCurrent, BLCurrent, BRCurrent};
        String[] namesArray   = {"FL Drive", "FR Drive", "BL Drive", "BR Drive"};

        for (int i = 0; i<currentArray.length; i++) {
            if ((currentArray[i] > (currentArray[0]) * 1.5) || (currentArray[i] > (currentArray[1]) * 1.5) || (currentArray[i] > (currentArray[2]) * 1.5) ||(currentArray[i] > (currentArray[3]) * 1.5)) {
                System.out.println(namesArray[i] + " has a high current of: " + currentArray[i]);
            }
        }
    }

    public static void checkFaultyRotateMotors(){
        double FLCurrent = powerDistribution.getCurrent(FL_ROTATE_ID);
        double FRCurrent = powerDistribution.getCurrent(FR_ROTATE_ID);
        double BLCurrent = powerDistribution.getCurrent(BL_ROTATE_ID);
        double BRCurrent = powerDistribution.getCurrent(BR_ROTATE_ID);

        double[] currentArray = {FLCurrent, FRCurrent, BLCurrent, BRCurrent};
        String[] namesArray   = {"FL Rotate", "FR Rotate", "BL Rotate", "BR Rotate"};

        for (int i = 0; i<currentArray.length; i++) {
            if ((currentArray[i] > (currentArray[0]) * 1.5) || (currentArray[i] > (currentArray[1]) * 1.5) || (currentArray[i] > (currentArray[2]) * 1.5) ||(currentArray[i] > (currentArray[3]) * 1.5)) {
                System.out.println(namesArray[i] + " has a high current of: " + currentArray[i]);
            }
        }
    }

}
