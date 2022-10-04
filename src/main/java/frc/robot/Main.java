// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private Main() {}

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

/**
 * The old Main.java that Charles had written.
 * <p>Temporarily placed here while I (Alex) figure out the best way to merge the two. 
 */
/*
public class Main {
  public static void main(String[] args) {
      AprilTag topRightCorner = new AprilTag(0, 10);
      AprilTag rightEdge = new AprilTag(-5, -5);

      double[] robot_coords = AprilTag.calculateRobotLocation(topRightCorner, 50, rightEdge, 50);

      // Length of 0 would mean error
      if (robot_coords.length == 2) {
          System.out.println(robot_coords[0] + " " + robot_coords[1]);
      }
  }
}
*/