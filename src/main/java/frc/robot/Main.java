// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables or initialization to this class
 * Do not modify this file except to change the parameter to startRobot()
 */
public final class Main {
    /**
     * Constructor for the Main class
     */
    private Main() {}

    /**
     * Main initialization function.
     *
     * <p>If you change your main robot class, change the parameter type.
     */
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}

// End of the main class