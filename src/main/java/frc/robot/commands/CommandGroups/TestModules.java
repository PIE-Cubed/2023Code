// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import frc.robot.Drive;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * The start of the TestModules class
 */
public class TestModules extends SequentialCommandGroup {
    /**
     * The constructor for the TestModules class
     */
    public TestModules(Drive drive) {
        //Adds commands to the group
        addCommands(
            new RunCommand(
                () ->
                drive.initWheelPowerTests()
            ),
            new RunCommand(
                () ->
                drive.testModuleRotation(0.00)
            ),
            new Delay(1.00),
            new RunCommand(
                () ->
                drive.testModuleRotation(Math.PI / 2)
            ),
            new Delay(1.00),
            new RunCommand(
                () ->
                drive.testModuleRotation(Math.PI)
            ),
            new Delay(1.00),
            new RunCommand(
                () ->
                drive.testModuleRotation(- Math.PI / 2)
            ),
            new Delay(1.00)
        );
    }
}

// End of the TestModules class