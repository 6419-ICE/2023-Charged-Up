// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.TrajectoryPaths;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// This auto is meant to drive straight onto the charging platform, and then balance on it
public class AutoEngageOnChargingStation extends SequentialCommandGroup {
  /* Creates a new Autonomous Program. */

  public AutoEngageOnChargingStation(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.sequence(
        new WaitCommand(.5),
        new TrajectoryCommand(driveSubsystem, TrajectoryPaths.trajectoryAutoEngageOnChargingStation())
      )
    );

  }
}
