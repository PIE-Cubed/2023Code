// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.networktables.*;

/**
 * Start of the CustomTables class
 */
public class CustomTables {
    // The FMSInfo Table and its entries
	private NetworkTable FMSInfo;
	private NetworkTableEntry isRedAlliance;
    
    // The TagInfo Table and its entries
    private NetworkTable TagInfo;
    private NetworkTableEntry time;
    private NetworkTableEntry bestResult;
    private NetworkTableEntry targetValid;
    private NetworkTableEntry BestResultId;
    private NetworkTableEntry detectionTime;

    // Singleton for CustomTables to ensure only one NetworkTables server is created
    private static CustomTables instance = null;
    public static synchronized CustomTables getInstance() {
        if (instance == null) {
            instance = new CustomTables();
        }

        return instance;
    }

    /**
     * The constructor for the CustomTables class
     */
    private CustomTables() {
        // Gets the default instance
        NetworkTableInstance ntInst = NetworkTableInstance.getDefault();

        // Creates the FMSInfo table and its entries
		FMSInfo = ntInst.getTable("FMSInfo");
		isRedAlliance = FMSInfo.getEntry("IsRedAlliance"); // boolean

        // Creates the TagInfo table and its entries
		TagInfo = ntInst.getTable("TagInfo");
		targetValid   = TagInfo.getEntry("tv");            // boolean
        time          = TagInfo.getEntry("time");          // double
        bestResult    = TagInfo.getEntry("BestResult");    // double[]
        BestResultId  = TagInfo.getEntry("BestResultId");  // double
        detectionTime = TagInfo.getEntry("DetectionTime"); // double
    }

    /****************************************************************************************** 
    *
    *    GETS VALUES FROM FMSINFO
    * 
    ******************************************************************************************/
    /**
     * Gets our alliance color from the FMS.
     * 
     * @return isRed
     */
    public boolean getIsRedAlliance() {
        return isRedAlliance.getBoolean(false);
    }


    /****************************************************************************************** 
    *
    *    GETS VALUES FROM TAGINFO
    * 
    ******************************************************************************************/
    /**
     * Determines if there are valid targets from the Jetson.
     * 
     * @return targetValid
     */
    public boolean getTargetValid() {
        return targetValid.getBoolean(false);
    }

    /**
     * Gets the time a detection was made from the Jetson.
     * 
     * @return detectionTime
     */
    public double getDetectionTime() {
        return detectionTime.getDouble(-1);
    }

    /**
     * Gets the tag id of the best detection from the Jetson.
     * 
     * @return bestTagID
     */
    public int getBestResultID() {
        return (int) BestResultId.getDouble(-1);
    }

    /**
     * Gets the best tag's pose from the Jetson.
     * 
     * @return bestTagPose
     */
    public Pose3d getBestResult() {
        // Gets the best camera to tag pose from the Jetson
        double[] defaultValue = {-1, 0, 0, 0, 0, 0, 0};
        double[] info = bestResult.getDoubleArray(defaultValue);

        // Assembles a Pose3d object from the double array
        Pose3d pose = new Pose3d(
            new Translation3d(info[1], info[2], info[3]),
            new Rotation3d(info[4], info[5], info[6])
        );

        return pose;
    }

    /****************************************************************************************** 
    *
    *    SETS VALUES IN TAGINFO
    * 
    ******************************************************************************************/
    /**
     * Sets the current time.
     * 
     * @param currTime
     */
    public void setTime(double currTime) {
        time.setDouble(currTime);
    }
}

// End of the CustomTables class