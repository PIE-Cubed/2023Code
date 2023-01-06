// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Start of the CustomTables class
 */
public class CustomTables {
    // Network Tables Instance
    private static NetworkTableInstance ntinst = NetworkTableInstance.getDefault();

    // FMSInfo Table
    private static NetworkTable FMSInfo = ntinst.getTable("FMSInfo");
    private static NetworkTableEntry isRedAlliance = FMSInfo.getEntry("IsRedAlliance");  // Boolean

    // RobotData Table
    private static NetworkTable RobotData = ntinst.getTable("RobotData");
    private static NetworkTableEntry gyroYaw    = RobotData.getEntry("GyroYaw");     // Double
    private static NetworkTableEntry detectTime = RobotData.getEntry("DetectTime");  // Double
    private static NetworkTableEntry FLRot      = RobotData.getEntry("FLRotation");  // Double
    private static NetworkTableEntry RLRot      = RobotData.getEntry("RLRotation");  // Double
    private static NetworkTableEntry FRRot      = RobotData.getEntry("FRRotation");  // Double
    private static NetworkTableEntry RRRot      = RobotData.getEntry("RRRotation");  // Double
    private static NetworkTableEntry FLVel      = RobotData.getEntry("FLVelocity");  // Double
    private static NetworkTableEntry RLVel      = RobotData.getEntry("RLVelocity");  // Double
    private static NetworkTableEntry FRVel      = RobotData.getEntry("FRVelocity");  // Double
    private static NetworkTableEntry RRVel      = RobotData.getEntry("RRVelocity");  // Double

    // TagInfo Table
    private static NetworkTable TagInfo = ntinst.getTable("TagInfo");
    private static NetworkTableEntry bestResult = TagInfo.getEntry("BestResult");  // Double[]

    /**
     * The constructor for the CustomTables class
     */
    private CustomTables() {
        // Purposfully blank beacuse all methods are static
    }

    /****************************************************************************************** 
    *
    *    SETTER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * 
     * @param FLVelocity
     * @param FLRotation
     */
    public static void setFLState(double FLVelocity, double FLRotation) {
        FLVel.setDouble(FLVelocity);
        FLRot.setDouble(FLRotation);
    }

    /**
     * 
     * @param RLVelocity
     * @param RLRotation
     */
    public static void setRLState(double RLVelocity, double RLRotation) {
        RLVel.setDouble(RLVelocity);
        RLRot.setDouble(RLRotation);
    }

    /**
     * 
     * @param FRVelocity
     * @param FRRotation
     */
    public static void setFRState(double FRVelocity, double FRRotation) {
        FRVel.setDouble(FRVelocity);
        FRRot.setDouble(FRRotation);
    }

    /**
     * 
     * @param RRVelocity
     * @param RRRotation
     */
    public static void setRRState(double RRVelocity, double RRRotation) {
        RRVel.setDouble(RRVelocity);
        RRRot.setDouble(RRRotation);
    }

    /**
     * 
     * @param Heading
     */
    public static void setGyroYaw(double yaw) {
        gyroYaw.setDouble(yaw);
    }

    /****************************************************************************************** 
    *
    *    GETTER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Returns the result with the least error.
     * 
     * @return bestPose
     */
    public static int getBestResultId() {
        // Gets the best entry from the Jetson 
        double[] defaultValue = {-1, 100, 0, 0, 0, 0, 0, 0};
        double[] resultArray  = bestResult.getDoubleArray(defaultValue);

        // Returns the tag id
        return (int)resultArray[0];
    }

    /**
     * Returns ALL results
     * 
     * @return Pose3d
     */
    public static Pose3d getBestResult() {
        // Gets the best entry from the Jetson 
        double[] defaultValue = {-1, 100, 0, 0, 0, 0, 0, 0};
        double[] resultArray  = bestResult.getDoubleArray(defaultValue);

        // Calulate a pose from the retrieved data
        Pose3d bestPose = poseFromArray(resultArray);

        // Returns the Pose3d
        return bestPose;
    }

    /**
     * Returns the time when a detection was last made
     * 
     * @return detectionTime
     */
    public static double getDetectionTime() {
        return detectTime.getDouble(-1.00);
    }

    /**
     * Determines if we are on the red alliance
     * @return isRed
     */
    public static boolean getRedAlliance() {
        return isRedAlliance.getBoolean(false);
    }

    /****************************************************************************************** 
    *
    *    HELPER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Generates a Pose3d object from previously a double array.
     * 
     * @param data A data array containing [timestamp, x, y, z, roll, pitch, yaw]
     * @return Pose3d
     */
    private static Pose3d poseFromArray(double[] data) {
        // Unpacks the data array
        //double tagNum    = data[0];
        //double error     = data[1];
        double x         = data[2];  // Meters
        double y         = data[3];  // Meters
        double z         = data[4];  // Meters
        double roll      = data[5];  // Radians
        double pitch     = data[6];  // Radians
        double yaw       = data[7];  // Radians

        // Creates the Pose3d components
        Rotation3d    rot = new Rotation3d(roll, pitch, yaw);
        Translation3d trans = new Translation3d(x, y, z);

        // Returns a constructed Pose3d
        return new Pose3d(trans, rot);
    }
}

// End of the CustomTables class