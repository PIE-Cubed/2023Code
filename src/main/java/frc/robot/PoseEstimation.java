// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;;

/**
 * Start of the PoseEstimation class
 */
public class PoseEstimation {
    // CONSTANTS
    // Dimensions of the camera to robot transform (inches)
    private double X_OFFSET_IN = 0;
    private double Y_OFFSET_IN = 0;
    private double Z_OFFSET_IN = 0;

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
    private double prevTime = -1;
    private Pose3d prevPose = new Pose3d();

    // Object creation
    private Drive drive;
    private CustomTables nTables;
    private AprilTagFieldLayout field;
    private SwerveDriveOdometry odometry;
    private SwerveDrivePoseEstimator visionEstimator;

    /**
     * The constructor for the PoseEstimation class
     * 
     * @param drive
     */
    public PoseEstimation(Drive drive) {
        // Instance creation
        this.drive   = drive;
        this.nTables = CustomTables.getInstance();
 
        // Default module positions
        SwerveModulePosition[] moduleStartPosition = new SwerveModulePosition[4];

        // Creates the odometry tracker
        odometry = new SwerveDriveOdometry(
            drive.swerveDriveKinematics,
            new Rotation2d(0),
            moduleStartPosition,
            new Pose2d()
        );

        // Defines the vision pose estimator's trust values (higher means less trusted)
        Matrix<N3, N1> odometryTrust = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01); // x, y, theta
        Matrix<N3, N1> visionTrust   = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01); // x, y, theta

        // Creates the vision pose tracker
        visionEstimator = new SwerveDrivePoseEstimator(
            drive.swerveDriveKinematics,
            new Rotation2d(),
            moduleStartPosition,
            new Pose2d(),
            odometryTrust,
            visionTrust
        );

        // Tries to load the field from a json
        try {
            field = new AprilTagFieldLayout("src/2023-chargedup.json");
        } catch (IOException ex) {
            System.out.println("Unable to open trajectory: " + ex.getStackTrace());
        }

        // Sets the origin depending on alliance color
        boolean color = nTables.getRedAlliance();
        if (color == false) {
            // We are on the Red Alliance
            field.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        }
        else {
            // We are on the Blue Alliance
            field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
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
        updateVisionEstimator();
    }

    /**
     * Updates the SwerveDriveOdometry.
     */
    public void updateOdometry() {
        // Compiles all the module positions
        SwerveModulePosition[] allModulePosition = getAllModulePositions();

        // Updates odometry
        odometry.update(
            new Rotation2d( drive.getHeading() ),
            allModulePosition
        );
    }

    /**
     * Updates the VisionEstimator.
     */
    public void updateVisionEstimator() {
        // Compiles all the module positions
        SwerveModulePosition[] allModulePosition = getAllModulePositions();

        // Updates the pose estimator (without vision)
        visionEstimator.update(
            new Rotation2d( drive.getHeading() ),
            allModulePosition
        );

        // Gets current values
        int     id     = nTables.getBestResultID();
        boolean tv     = nTables.getTargetValid();
        Pose3d detPose = nTables.getBestResult();
        double detTime = nTables.getDetectionTime();

        // Adds the vision measurement if it hasn't been calculated yet
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
                    new Rotation3d(roll, pitch, yaw)
                );

                // Gets the camera's pose relative to the tag
                Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

                // Tranforms the camera's pose to the robot's center
                Pose3d measurement = camPose.transformBy(CAMERA_TO_ROBOT.inverse());

                // Adds the vision measurement
                visionEstimator.addVisionMeasurement(
                    measurement.toPose2d(),
                    detTime
                );
            }
        }
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
        resetVisionEstimator(pose);
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
            pose.getRotation(),
            allPositiions,
            pose
        );
    }

    /**
     * Resets the VisionEstimator to a defined position and rotation.
     * 
     * @param pose
     */
    private void resetVisionEstimator(Pose2d pose) {
        // Gets the module positions
        SwerveModulePosition[] allPositiions = getAllModulePositions();

        // Resets the SwerveOdometry
        visionEstimator.resetPosition(
            pose.getRotation(),
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
     * Gets the floorPose as calulated by the VisionEstimator.
     * 
     * @return The robot's floor pose
     */
    public Pose2d getVisionPose() {
        return visionEstimator.getEstimatedPosition();
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