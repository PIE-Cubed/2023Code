// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;;

/**
 * Start of the PoseEstimation class
 */
public class PoseEstimation {
    // CONSTANTS
    // Dimensions of the camera to robot transform (inches)
    private double X_OFFSET_IN = -12;
    private double Y_OFFSET_IN = -6;
    private double Z_OFFSET_IN = 19 + 3/8;

    // Dimensions of the camera to robot transform (meters)
    private double X_OFFSET_M = Units.inchesToMeters(X_OFFSET_IN);
    private double Y_OFFSET_M = Units.inchesToMeters(Y_OFFSET_IN);
    private double Z_OFFSET_M = Units.inchesToMeters(Z_OFFSET_IN);

    // Defines the camera to robot transform
    private Transform3d CAMERA_TO_ROBOT = new Transform3d(
        new Translation3d(X_OFFSET_M, Y_OFFSET_M, Z_OFFSET_M),
        new Rotation3d(0, 0, 0)
    );

    // Variables
    private double prevTime            = -1;
    private Pose3d prevPose            = new Pose3d();
    private long   lastAprilTagReading = 0;

    public boolean updateVision = true;

    // Object creation
    private Drive drive;
    private CustomTables nTables;
    private AprilTagFieldLayout field;
    private SwerveDriveOdometry odometry;

    /**
     * The constructor for the PoseEstimation class
     * 
     * @param drive
     */
    public PoseEstimation(Drive drive) {
        // Instance creation
        this.drive   = drive;
        this.nTables = CustomTables.getInstance();
 
        // Starting module positions
        SwerveModulePosition[] moduleStartPosition = getAllModulePositions();

        // Creates the odometry tracker
        odometry = new SwerveDriveOdometry(
            drive.swerveDriveKinematics,
            new Rotation2d( drive.getYawAdjusted() ),
            moduleStartPosition,
            new Pose2d(0, 0, new Rotation2d(-Math.PI))
        );

        // Tries to load the AprilTag positions from a json
        try {
            field = new AprilTagFieldLayout("/home/lvuser/2023-chargedup.json");
        } catch (IOException ex) {
            System.out.println("Unable to open field json: " + ex.getStackTrace());
        }

        // Sets the origin depending on alliance color
        boolean color = nTables.getIsRedAlliance();
        if (color == false) {
            // We are on the Blue Alliance
            field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        }
        else {
            // We are on the Red Alliance
            field.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        }
    }

    /****************************************************************************************** 
    *
    *    UPDATE FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Updates both Pose Estimators.
     */
    public void updatePoseTrackers() {
        updateOdometry();
    }

    /**
     * Updates the SwerveDriveOdometry.
     */
    public void updateOdometry() {
        // Compiles all the module positions
        SwerveModulePosition[] allModulePosition = getAllModulePositions();

        // Updates odometry
        odometry.update(
            new Rotation2d( MathUtil.angleModulus(drive.getYawAdjusted()) ),
            allModulePosition
        );
    }


    /****************************************************************************************** 
    *
    *    RESETTER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Resets both Pose Estimators to a defined position and rotation.
     * 
     * @param pose
     */
    public void resetPoseTrackers(Pose2d pose) {
        resetOdometry(pose);
    }

    /**
     * Resets the SwerveDriveOdometry to a defined position and rotation.
     * 
     * @param pose
     */
    private void resetOdometry(Pose2d pose) {
        // Gets the module positions
        SwerveModulePosition[] allPositiions = getAllModulePositions();

        // Resets the SwerveOdometry
        odometry.resetPosition(
            new Rotation2d(MathUtil.angleModulus(drive.getYawAdjusted())),
            allPositiions,
            pose
        );
    }


    /****************************************************************************************** 
    *
    *    GETTER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Gets the floorPose as calulated by the SwerveDriveOdometry.
     * 
     * @return The robot's floor pose
     */
    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    /**
     * 
     * @return
     */
    public Pose2d getPose() {
        // Gets current values
        int                    id                = nTables.getBestResultID();
        boolean                tv                = nTables.getTargetValid();
        Pose3d                 detPose           = nTables.getBestResult();
        double                 detTime           = nTables.getDetectionTime();

        // Adds the vision measurement if it hasn't been calculated and a target is visible
       /*  TJM  I think you need to use prevPose.equals(detPose)   */
        if ((prevTime != detTime) && (prevPose != detPose) && (tv == true)) {
            // Sets the prev variables
            prevTime = detTime;
            prevPose = detPose;

            // Adds the vision measurement if the tag id is valid
            if ((id != -1) && ((id > 0) && (id <= 8))) {
                // Gets the target's pose on the field
                Pose3d targetPose = field.getTagPose(id).get();

                // Extracts the x, y, and z distances
                double x = detPose.getTranslation().getX();
                double y = detPose.getTranslation().getY();
                double z = detPose.getTranslation().getZ();

                // Extracts the roll, pitch, and yaw angles
                double roll  = detPose.getRotation().getX();
                double pitch = detPose.getRotation().getY();
                double yaw   = detPose.getRotation().getZ();

                // Creates the relative pose
                Transform3d camToTarget = new Transform3d(
                    new Translation3d(x, y, z),
                    new Rotation3d(roll, pitch, Math.PI - yaw)
                );

                // Gets the camera's pose relative to the tag
                Pose3d camPose = targetPose.transformBy(camToTarget);

                // Tranforms the camera's pose to the robot's center
                Pose3d measurement = camPose.transformBy(CAMERA_TO_ROBOT);

                // Resets the odometry to the vision estimate
                if (x < Units.feetToMeters(10) && updateVision) {
                    System.out.println("Updating vision (should not be here if moving)");
                    lastAprilTagReading = System.currentTimeMillis();
                    resetOdometry(measurement.toPose2d());
                }
                else {
                    System.out.println("nvm we're not updating");
                }

                return getOdometryPose();
            }
        }

        return getOdometryPose();
    }

    /**
     * Checks if we had an April Tag update within the last 5 seconds
     * @return
     */
    public boolean recentAprilTag() {
        return (System.currentTimeMillis() < lastAprilTagReading + 5000);
    }


    /****************************************************************************************** 
    *
    *    HELPER FUNCTIONS
    * 
    ******************************************************************************************/
    /**
     * Gets the position of all four SwerveModules.
     * 
     * @return The position of all four wheels
     */
    private SwerveModulePosition[] getAllModulePositions() {
        // Creates a SwerveModlePosition array with all the wheels in it
        SwerveModulePosition[] allPositions = {
            drive.getFLPosition(),
            drive.getFRPosition(),
            drive.getBLPosition(),
            drive.getBRPosition()
        };

        return allPositions;
    }
}

// End of the PoseEstimation class