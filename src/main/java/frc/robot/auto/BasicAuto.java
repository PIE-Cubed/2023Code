// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import frc.robot.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * The start of the BasicAuto class
 */
public class BasicAuto extends SequentialCommandGroup {
    /**
     * The constructor for the BasicAuto class
     */
    public BasicAuto(Drive drive, PoseEstimation position, double delaySec) {
        // Position array {x, y, rotation}
        double[][] coorList = {
            {0, 0, 0},
            {1, 0, 0},
            {2, 0, 0}
        };

        //Adds commands to the group
        addCommands(
            new AutoDrive(drive, position, coorList)
        );
    }
}

// End of the BasicAuto class