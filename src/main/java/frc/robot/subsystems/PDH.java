// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Start of the PDH class
 */
public class PDH extends SubsystemBase {
    // Drive Motor ID
    private final int FL_DRIVE = 10;
    private final int RL_DRIVE = 12;
    private final int FR_DRIVE = 14;
    private final int RR_DRIVE = 16;

    // Rotate Motor ID
    private final int FL_ROTATE = 11;
    private final int RL_ROTATE = 13;
    private final int FR_ROTATE = 17;
    private final int RR_ROTATE = 15;

    // Object Creation
    private PowerDistribution powerDistribution;

    /**
     * The constructor for the PDH class
     */
    public PDH () {
        // Creates an instance of PowerDistribution
        powerDistribution = new PowerDistribution();
    }

    /**
     * checkFaultyDriveMotors()
     * <p>Checks for current spikes in the drive motors
     * <p>Prints the results to the RioLog
     */
    public void checkFaultyDriveMotors() {
        double FLCurrent = powerDistribution.getCurrent(FL_DRIVE);
        double FRCurrent = powerDistribution.getCurrent(FR_DRIVE);
        double RLCurrent = powerDistribution.getCurrent(RL_DRIVE);
        double RRCurrent = powerDistribution.getCurrent(RR_DRIVE);

        double[] currentArray = {FLCurrent, FRCurrent, RLCurrent, RRCurrent};
        String[] namesArray   = {"FL Drive", "FR Drive", "BL Drive", "BR Drive"};

        for (int i = 0; i < currentArray.length; i++) {
            if ((currentArray[i] > (currentArray[0]) * 1.5) || (currentArray[i] > (currentArray[1]) * 1.5) || (currentArray[i] > (currentArray[2]) * 1.5) ||(currentArray[i] > (currentArray[3]) * 1.5)) {
                System.out.println(namesArray[i] + " has a high current of: " + currentArray[i]);
            }
        }
    }

    /**
     * checkFaultyRotateMotors()
     * <p>Checks for current spikes in the rotate motors
     * <p>Prints results to the RioLog
     */
    public void checkFaultyRotateMotors() {
        double FLCurrent = powerDistribution.getCurrent(FL_ROTATE);
        double FRCurrent = powerDistribution.getCurrent(FR_ROTATE);
        double RLCurrent = powerDistribution.getCurrent(RL_ROTATE);
        double RRCurrent = powerDistribution.getCurrent(RR_ROTATE);

        double[] currentArray = {FLCurrent, FRCurrent, RLCurrent, RRCurrent};
        String[] namesArray   = {"FL Rotate", "FR Rotate", "BL Rotate", "BR Rotate"};

        for (int i = 0; i < currentArray.length; i++) {
            if ((currentArray[i] > (currentArray[0]) * 1.5) || (currentArray[i] > (currentArray[1]) * 1.5) || (currentArray[i] > (currentArray[2]) * 1.5) ||(currentArray[i] > (currentArray[3]) * 1.5)) {
                System.out.println(namesArray[i] + " has a high current of: " + currentArray[i]);
            }
        }
    }
}

// End of the PDH class