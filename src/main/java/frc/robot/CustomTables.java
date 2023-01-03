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

    // RobotData Table
    private static NetworkTable RobotData = ntinst.getTable("RobotData");
    private static NetworkTableEntry gyroYaw        = RobotData.getEntry("GyroYaw");     // Double
    private static NetworkTableEntry currentTimeSec = RobotData.getEntry("CurrentTime"); // Double
    private static NetworkTableEntry detectTime     = RobotData.getEntry("DetectTime");  // Double
    private static NetworkTableEntry FLRot          = RobotData.getEntry("FLRotation");  // Double
    private static NetworkTableEntry RLRot          = RobotData.getEntry("RLRotation");  // Double
    private static NetworkTableEntry FRRot          = RobotData.getEntry("FRRotation");  // Double
    private static NetworkTableEntry RRRot          = RobotData.getEntry("RRRotation");  // Double
    private static NetworkTableEntry FLVel          = RobotData.getEntry("FLVelocity");  // Double
    private static NetworkTableEntry RLVel          = RobotData.getEntry("RLVelocity");  // Double
    private static NetworkTableEntry FRVel          = RobotData.getEntry("FRVelocity");  // Double
    private static NetworkTableEntry RRVel          = RobotData.getEntry("RRVelocity");  // Double

    // FMSInfo Table
    private static NetworkTable FMSInfo = ntinst.getTable("FMSInfo");
    private static NetworkTableEntry isRedAlliance = FMSInfo.getEntry("IsRedAlliance");  // Boolean

    // TagInfo Table
    private static NetworkTable TagInfo = ntinst.getTable("TagInfo");
    private static NetworkTableEntry tVecsDetected       = TagInfo.getEntry("TranslationVectors");  // String[]
    private static NetworkTableEntry rMatrixesDetected   = TagInfo.getEntry("RotationMatrixes");  // String[]
    private static NetworkTableEntry eulerAnglesDetected = TagInfo.getEntry("EulerAngles");  // String[]

    // Create an AllTags Tables and its Entries
    private static NetworkTable AllTags    = ntinst.getTable("AllTags");
    private static NetworkTableEntry tag0  = AllTags.getEntry("Tag0");   // Double[]
    private static NetworkTableEntry tag1  = AllTags.getEntry("Tag1");   // Double[]
    private static NetworkTableEntry tag2  = AllTags.getEntry("Tag2");   // Double[]
    private static NetworkTableEntry tag3  = AllTags.getEntry("Tag3");   // Double[]
    private static NetworkTableEntry tag4  = AllTags.getEntry("Tag4");   // Double[]
    private static NetworkTableEntry tag5  = AllTags.getEntry("Tag5");   // Double[]
    private static NetworkTableEntry tag6  = AllTags.getEntry("Tag6");   // Double[]
    private static NetworkTableEntry tag7  = AllTags.getEntry("Tag7");   // Double[]
    private static NetworkTableEntry tag8  = AllTags.getEntry("Tag8");   // Double[]
    private static NetworkTableEntry tag9  = AllTags.getEntry("Tag9");   // Double[]
    private static NetworkTableEntry tag10 = AllTags.getEntry("Tag10");  // Double[]
    private static NetworkTableEntry tag11 = AllTags.getEntry("Tag11");  // Double[]
    private static NetworkTableEntry tag12 = AllTags.getEntry("Tag12");  // Double[]
    private static NetworkTableEntry tag13 = AllTags.getEntry("Tag13");  // Double[]
    private static NetworkTableEntry tag14 = AllTags.getEntry("Tag14");  // Double[]
    private static NetworkTableEntry tag15 = AllTags.getEntry("Tag15");  // Double[]
    private static NetworkTableEntry tag16 = AllTags.getEntry("Tag16");  // Double[]
    private static NetworkTableEntry tag17 = AllTags.getEntry("Tag17");  // Double[]
    private static NetworkTableEntry tag18 = AllTags.getEntry("Tag18");  // Double[]
    private static NetworkTableEntry tag19 = AllTags.getEntry("Tag19");  // Double[]
    private static NetworkTableEntry tag20 = AllTags.getEntry("Tag20");  // Double[]
    private static NetworkTableEntry tag21 = AllTags.getEntry("Tag21");  // Double[]
    private static NetworkTableEntry tag22 = AllTags.getEntry("Tag22");  // Double[]
    private static NetworkTableEntry tag23 = AllTags.getEntry("Tag23");  // Double[]
    private static NetworkTableEntry tag24 = AllTags.getEntry("Tag24");  // Double[]
    private static NetworkTableEntry tag25 = AllTags.getEntry("Tag25");  // Double[]
    private static NetworkTableEntry tag26 = AllTags.getEntry("Tag26");  // Double[]
    private static NetworkTableEntry tag27 = AllTags.getEntry("Tag27");  // Double[]
    private static NetworkTableEntry tag28 = AllTags.getEntry("Tag28");  // Double[]
    private static NetworkTableEntry tag29 = AllTags.getEntry("Tag29");  // Double[]

    // Variables
    private static int i = 0;
    private static int prevError = 0;
    private static int leastErrorId = 0;

    /**
     * The constructor for the CustomTables class
     */
    private CustomTables() {
        // Purposfully blank beacuse all methods are static
    }

    /**
     * Returns the result with the least error.
     * 
     * @return bestPose
     */
    public static Pose3d getBestResult() {
        // Gets all entries
        Pose3d[] allPose = getLatestResults();

        return allPose[leastErrorId];
    }

    /**
     * Returns ALL results
     * 
     * @return allPose
     */
    public static Pose3d[] getLatestResults() {
        // Creates two blank arrarys
        double[] defaultValue = {-1, 0, 0, 0, 0, 0, 0};
        Pose3d[] allPose = new Pose3d[30];

        // Fills the allPose array
        allPose[0]  = poseFromArray(tag0.getDoubleArray(defaultValue));
        allPose[1]  = poseFromArray(tag1.getDoubleArray(defaultValue));
        allPose[2]  = poseFromArray(tag2.getDoubleArray(defaultValue));
        allPose[3]  = poseFromArray(tag3.getDoubleArray(defaultValue));
        allPose[4]  = poseFromArray(tag4.getDoubleArray(defaultValue));
        allPose[5]  = poseFromArray(tag5.getDoubleArray(defaultValue));
        allPose[6]  = poseFromArray(tag6.getDoubleArray(defaultValue));
        allPose[7]  = poseFromArray(tag7.getDoubleArray(defaultValue));
        allPose[8]  = poseFromArray(tag8.getDoubleArray(defaultValue));
        allPose[9]  = poseFromArray(tag9.getDoubleArray(defaultValue));
        allPose[10] = poseFromArray(tag10.getDoubleArray(defaultValue));
        allPose[11] = poseFromArray(tag11.getDoubleArray(defaultValue));
        allPose[12] = poseFromArray(tag12.getDoubleArray(defaultValue));
        allPose[13] = poseFromArray(tag13.getDoubleArray(defaultValue));
        allPose[14] = poseFromArray(tag14.getDoubleArray(defaultValue));
        allPose[15] = poseFromArray(tag15.getDoubleArray(defaultValue));
        allPose[16] = poseFromArray(tag16.getDoubleArray(defaultValue));
        allPose[17] = poseFromArray(tag17.getDoubleArray(defaultValue));
        allPose[18] = poseFromArray(tag18.getDoubleArray(defaultValue));
        allPose[19] = poseFromArray(tag19.getDoubleArray(defaultValue));
        allPose[20] = poseFromArray(tag20.getDoubleArray(defaultValue));
        allPose[21] = poseFromArray(tag21.getDoubleArray(defaultValue));
        allPose[22] = poseFromArray(tag22.getDoubleArray(defaultValue));
        allPose[23] = poseFromArray(tag23.getDoubleArray(defaultValue));
        allPose[24] = poseFromArray(tag24.getDoubleArray(defaultValue));
        allPose[25] = poseFromArray(tag25.getDoubleArray(defaultValue));
        allPose[26] = poseFromArray(tag26.getDoubleArray(defaultValue));
        allPose[27] = poseFromArray(tag27.getDoubleArray(defaultValue));
        allPose[28] = poseFromArray(tag28.getDoubleArray(defaultValue));
        allPose[29] = poseFromArray(tag29.getDoubleArray(defaultValue));

        return allPose;
    }

    /**
     * Returns the coordinate data calulated by the Jetson.
     * <p> The x, y, and z are the translation vectors in meters
     * @return coor: Each entry in the array is [tagId, x, y, z]
     */
    public static double[][] getCoordinatesAll() {
        // Gets the data from the TranslationVectors entry
        String[] defaultValue = {"1, 2, 3, 4,", "5, 6, 7, 8,"};
        String[] recievedData = tVecsDetected.getStringArray(defaultValue);

        // Variables
        int i = 0;
        int j = 0;
        double[][] returnData = new double[recievedData.length][4];

        // Loops through all available data
        for (String data: recievedData) {
            // Splits the string into parts using commas as a seperator
            String[] parts = data.split("\\,");

            // Loops through each part
            for (String part: parts) {
                // Converts each part to a double
                returnData[i][j] = Double.parseDouble(part);

                // Increases the value of j
                j++;
            }

            // Increment i and reset j
            i++;
            j = 0;
        }

        // Returns the data
        return returnData;
    }

    /**
     * Returns the Rotation Matrixes calulated by the Jetson.
     * <p>The Rotation Matrix is a 3x3 array that can be created from the last 9 entries in this array
     * @return rMatrix: Each entry in the array is [tagId, e0, e1, e2, e3, e4, e5, e6, e7, e8]
     */
    public static double[][] getRotationMatrixesAll() {
        // Gets the data from the coordinatesDetected entry
        String[] defaultValue = {"1, 2, 3, 4, 5, 6, 7, 8, 9, 10,", "11, 12, 13, 14, 15, 16, 17, 18, 19,"};
        String[] recievedData = rMatrixesDetected.getStringArray(defaultValue);

        // Variables
        int i = 0;
        int j = 0;
        double[][] returnData = new double[recievedData.length][10];

        // Loops through all available data
        for (String data: recievedData) {
            // Splits the string into parts using commas as a seperator
            String[] parts = data.split("\\,");

            // Loops through each part
            for (String part: parts) {
                // Converts each part to a double
                returnData[i][j] = Double.parseDouble(part);

                // Increases the value of j
                j++;
            }

            // Increment i and reset j
            i++;
            j = 0;
        }

        // Returns the data
        return returnData;
    }

    /**
     * Returns the Euler Angles as calulated by the Jetson.
     * <p>The Euler Angles will be measured in radians
     * @return eulerAngles: Each entry in the array is [tagId, pitch, yaw, roll]
     */
    public static double[][] getEulerAnglesAll() {
        // Gets the data from the coordinatesDetected entry
        String[] defaultValue = {"1, 2, 3, 4,", "5, 6, 7, 8,"};
        String[] recievedData = eulerAnglesDetected.getStringArray(defaultValue);

        // Variables
        int i = 0;
        int j = 0;
        double[][] returnData = new double[recievedData.length][4];

        // Loops through all available data
        for (String data: recievedData) {
            // Splits the string into parts using commas as a seperator
            String[] parts = data.split("\\,");

            // Loops through each part
            for (String part: parts) {
                // Converts each part to a double
                returnData[i][j] = Double.parseDouble(part);

                // Increases the value of j
                j++;
            }

            // Increment i and reset j
            i++;
            j = 0;
        }

        // Returns the data
        return returnData;
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

    /**
     * Sets the current time for the coprocessors
     */
    public static void setTimeSec() {
        long time = System.currentTimeMillis() / 1000;
        currentTimeSec.setDouble((double)time);
    }

    /****************************************************************************************** 
    *
    *    GETTER FUNCTIONS
    * 
    ******************************************************************************************/
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
        double error     = data[0];
        double x         = data[1];  // Meters
        double y         = data[2];  // Meters
        double z         = data[3];  // Meters
        double roll      = data[4];  // Radians
        double pitch     = data[5];  // Radians
        double yaw       = data[6];  // Radians

        // Calculates the entry with the least error
        calcLeastErrorId(error);

        // Creates the Pose3d components
        Rotation3d    rot = new Rotation3d(roll, pitch, yaw);
        Translation3d trans = new Translation3d(x, y, z);

        // Returns a constructed Pose3d
        return new Pose3d(trans, rot);
    }

    /**
     * Calulates the id of the entry with the least error
     */
    private static void calcLeastErrorId(double error) {
        // Resets i
        if (i >= 30) {
            i = 0;
        }

        // Calculates the lowest error
        if (error < prevError) {
            leastErrorId = i;
        }

        // Increments i
        i++;
    }
}

// End of the CustomTables class