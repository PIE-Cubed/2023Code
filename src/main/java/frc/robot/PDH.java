// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * Start of the PDH class
 */
public class PDH {
	// CONSTANTS
	private static final int FL_ROTATE_ID = 0;
	private static final int FL_DRIVE_ID  = 0;
	private static final int FR_ROTATE_ID = 0;
	private static final int FR_DRIVE_ID  = 1;
	private static final int BL_ROTATE_ID = 0;
	private static final int BL_DRIVE_ID  = 2;
	private static final int BR_ROTATE_ID = 0;
	private static final int BR_DRIVE_ID  = 3;
	
	private static final int ITERATIONS_PER_HOUR = 50 * 3600;
	
	private static final int BATTERY_STORAGE = 216; // 18 Ah * 12 V - test this
	
	// Variables
	private static double FL_DRIVE_AMP_HOUR = 0;
	private static double FR_DRIVE_AMP_HOUR = 0;
	private static double BL_DRIVE_AMP_HOUR = 0;
	private static double BR_DRIVE_AMP_HOUR = 0;
	private static boolean driveErrorReported = false;
	
	private static double FL_ROTATE_AMP_HOUR = 0;
	private static double FR_ROTATE_AMP_HOUR = 0;
	private static double BL_ROTATE_AMP_HOUR = 0;
	private static double BR_ROTATE_AMP_HOUR = 0;
	private static boolean rotateErrorReported = false;


	// Object Creation
	private static PowerDistribution powerDistribution;

	/**
	 * Constructor
	 */
	public PDH () {
		// Creates an instance of PowerDistribution
		powerDistribution = new PowerDistribution();
		powerDistribution.clearStickyFaults();
	}

	/**
	 * checkFaultyDriveMotors()
	 * Checks for prolonged excess power draw in the drive motors
	 * <p>Prints the results to the RioLog
	 */
	public static void checkFaultyDriveMotors() {
		double FL_DRIVE_AMP_HOUR += powerDistribution.getCurrent(FL_DRIVE_ID) / ITERATIONS_PER_HOUR;
		double FR_DRIVE_AMP_HOUR += powerDistribution.getCurrent(FR_DRIVE_ID) / ITERATIONS_PER_HOUR;
		double BL_DRIVE_AMP_HOUR += powerDistribution.getCurrent(BL_DRIVE_ID) / ITERATIONS_PER_HOUR;
		double BR_DRIVE_AMP_HOUR += powerDistribution.getCurrent(BR_DRIVE_ID) / ITERATIONS_PER_HOUR;

		double[] drawArray  = {FL_DRIVE_AMP_HOUR, FR_DRIVE_AMP_HOUR, BL_DRIVE_AMP_HOUR, BR_DRIVE_AMP_HOUR};
		String[] namesArray = {"FL Drive", "FR Drive", "BL Drive", "BR Drive"};

		double averageDraw = (FL_DRIVE_AMP_HOUR + FR_DRIVE_AMP_HOUR + BL_DRIVE_AMP_HOUR + BR_DRIVE_AMP_HOUR) / 4;
		
		if (driveErrorReported == false) {
			for (int i = 0; i < drawArray.length; i++) {
				if ((currentArray[i] > averageDraw * 1.5) {
					System.out.println(namesArray[i] + " has a high power draw.");
					driveErrorReported = true;
				}
			}
		}
	}

	/**
	 * checkFaultyRotateMotors()
	 * Checks for prolonged excess power draw in the rotate motors
	 * <p>Prints results to the RioLog
	 */
	public static void checkFaultyRotateMotors() {
		double FL_ROTATE_AMP_HOUR += powerDistribution.getCurrent(FL_ROTATE_ID) / ITERATIONS_PER_HOUR;
		double FR_ROTATE_AMP_HOUR += powerDistribution.getCurrent(FR_ROTATE_ID) / ITERATIONS_PER_HOUR;
		double BL_ROTATE_AMP_HOUR += powerDistribution.getCurrent(BL_ROTATE_ID) / ITERATIONS_PER_HOUR;
		double BR_ROTATE_AMP_HOUR += powerDistribution.getCurrent(BR_ROTATE_ID) / ITERATIONS_PER_HOUR;

		double[] drawArray  = {FL_ROTATE_AMP_HOUR, FR_ROTATE_AMP_HOUR, BL_ROTATE_AMP_HOUR, BR_ROTATE_AMP_HOUR};
		String[] namesArray = {"FL Rotate", "FR Rotate", "BL Rotate", "BR Rotate"};

		double averageDraw = (FL_ROTATE_AMP_HOUR + FR_ROTATE_AMP_HOUR + BL_ROTATE_AMP_HOUR + BR_ROTATE_AMP_HOUR) / 4;
		
		if (rotateErrorReported == false) {
			for (int i = 0; i < drawArray.length; i++) {
				if ((currentArray[i] > averageDraw * 1.5) {
					System.out.println(namesArray[i] + " has a high power draw.");
					rotateErrorReported = true;
				}
			}
		}
	}
		
	public static void showBatteryPercent() {
		double totalJoulesUsed    = powerDistribution.getTotalEnergy();
		double totalWattHoursUsed = totalJoulesUsed / 3600;
		double percentRemaining   = ((BATTERY_STORAGE - totalWattHoursUsed) / BATTERY_STORAGE) * 100;
	}
}

// End of the PDH class
