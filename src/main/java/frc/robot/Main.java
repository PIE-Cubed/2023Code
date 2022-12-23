// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Start of the Main class
 * <p>Do NOT add any variables or initialization to this class.
 * <p>Do NOT modify this file except to change the parameter class in startRobot().
 */
public final class Main {
  /**
   * Constructor
   */
  private Main() {
    // Nothing yet...
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }

}

// End of the Main class