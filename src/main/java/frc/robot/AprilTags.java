package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

/**
 * Start of the AprilTags class
 */
public class AprilTags {
    // CONSTANTS
    private final double FIELD_WIDTH  = EaseOfUse.feetToMeters(26.0 + (7.0 / 12.0));
    private final double FIELD_LENGTH = EaseOfUse.feetToMeters(54.0 + (1.0 / 12.0));

    // Creates a blank list
    private List<AprilTag> allTags;

    // Object creation
    private AprilTag apriltag;
    private AprilTagFieldLayout apriltagFieldLayout;

    /**
     * The constructor for the AprilTags class.
     */
    public AprilTags(boolean isRed) {
        // Stores the Pose of all available AprilTags in the allTags list
        addTag(0, null);

        // Generates an AprilTagField
        apriltagFieldLayout = new AprilTagFieldLayout(allTags, FIELD_LENGTH, FIELD_WIDTH);

        // Adjust the origin depending on alliance
        if (isRed == true) {
            // Adjusts the origin to the right side wall of the red alliance
            apriltagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        }
        else {
            // Adjusts the origin to the right side wall of the blue alliance
            apriltagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        }
    }

    /**
     * addTag()
     * <p>Creates an AprilTag and adds it to the allTags list
     * @param id
     * @param pose
     */
    private void addTag(int id, Pose3d pose) {
        // Creates a new AprilTag
        apriltag = new AprilTag(id, pose);

        // Adds that tag to the master list
        allTags.add(apriltag);
    }
}

// End of the AprilTags Class