package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.networktables.*;

import org.photonvision.*;

/**
 * Start of the PhotonVision Class
 */
public class PhotonVision {
    // Network table creation
    private NetworkTable photonvisionEntries;

    // Network table entries
    private NetworkTableEntry hasTarget;
    private NetworkTableEntry targetPose;
    private NetworkTableEntry targetArea;
    private NetworkTableEntry pipelineIndex;
    private NetworkTableEntry ledMode;
    private NetworkTableEntry driverMode;
    private NetworkTableEntry rawBytes;

    // Enumerators
    public static enum LEDState {
        ON,
        OFF;
    }

    /**
     * Constructor
     */
    public PhotonVision() {
        // Network Table
        photonvisionEntries = NetworkTableInstance.getDefault().getTable("photonvision");

        // Network Table Entries
        hasTarget     = photonvisionEntries.getEntry("hasTarget");     // boolean
        targetPose    = photonvisionEntries.getEntry("targetPose");    // double[x, y, rotation in degrees]
        targetArea    = photonvisionEntries.getEntry("targetArea");    // double
        pipelineIndex = photonvisionEntries.getEntry("pipelineIndex"); // int
        ledMode       = photonvisionEntries.getEntry("ledMode");       // int
        driverMode    = photonvisionEntries.getEntry("driverMode");    // boolean
        rawBytes      = photonvisionEntries.getEntry("rawBytes");      // byte[]
    }

    /**
     * 
     * @return
     */
    public double aprilTagDetection() {
        return 0;
    }

    /**
     * Gets the value of hasTarget
     * @return validTarget
     */
    public boolean getHasTarget() {
        return hasTarget.getBoolean(false);
    }

    /**
     * Gets the Target's Position
     * @return targetPose
     */
    public double[] getTargetPose() {
        //Creates a default array
        double[] defaultArray = {0.0, 0.0, 0.0};

        // Returns the targetPose
        return targetPose.getDoubleArray(defaultArray);
    }

    /**
     * Returns the X Offset
     * @return xOffset
     */
    public double getXOffset() {
        // Gets the targetPose
        double[] pose = getTargetPose();
        
        // Return the X of the pose
        return pose[0];
    }

    /**
     * Returns the Y Offset
     * @return YOffset
     */
    public double getYOffset() {
        // Gets the targetPose
        double[] pose = getTargetPose();
        
        // Return the X of the pose
        return pose[1];
    }

    /**
     * Returns the Rotational Offset
     * @return rotationalOffset
     */
    public double getRotOffset() {
        // Gets the targetPose
        double[] pose = getTargetPose();
        
        // Return the X of the pose
        return pose[2];
    }

    /**
     * Gets the value of targetArea
     * @return area
     */
    public double getTargetArea() {
        return targetArea.getDouble(0.00);
    }

    /**
     * Gets the raw byte value
     * @return timeStamp
     * @warn I have no idea what that data this method returns looks like
     */
    public byte[] getRawBytes() {
        // Creates a default array
        byte[] defaultArray = {0, 1};

        // Returns the Timestamp
        return rawBytes.getRaw(defaultArray);
    }

    /**
     * Chanegs the current PhotonVision pipeline
     * @param pipelineName
     */
    public void changePipeline() {
        pipelineIndex.setNumber(0);
    }

    /**
     * Changes the PhotonVision LED mode
     * @param LEDMode
     */
    public void changeledMode(LEDState mode) {
        // Makes it easier to change the LED mode
        if (mode == LEDState.ON) {
            // Sets limelight to on
            ledMode.setNumber(1);
        }
        else if (mode == LEDState.OFF) {
            // Sets limelight to off
            ledMode.setNumber(0);
        }
        else {
            // Sets limelight to the pipeline default
            ledMode.setNumber(-1);
        }
    }

    /**
     * Changes the PhotonVision Driver Mode
     */
    public void changeDriverMode(boolean driverModeEnabled) {
        driverMode.setBoolean(driverModeEnabled);
    }

}

// End of the PhotonVision class