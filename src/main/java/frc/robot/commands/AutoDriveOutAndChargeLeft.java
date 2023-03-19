// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.TrajectoryPaths;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveOutAndChargeLeft extends SequentialCommandGroup {
  /** Creates a new Autonomous Program. */

  public AutoDriveOutAndChargeLeft(DriveSubsystem driveSubsystem,  BoolSupplierDriveUntilAngle boolSupplier) {
    RunnableAutoDriveUntilAngle AutoDriveRunnable =  new RunnableAutoDriveUntilAngle(driveSubsystem, boolSupplier, true);
   
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.sequence( 
        // Drives a trajectory meant to go forward and then left, while also rotating to 90 degrees
        new TrajectoryCommand(driveSubsystem, TrajectoryPaths.trajectoryAutoDriveOutLeft()),
        // Resets the Runcommand to allow it to run again (if the code has not been deployed multiple times)
        new InstantCommand(() -> AutoDriveRunnable.resetRunnable()),
        // Runs the runnable AutoDriveRunnable, which drives sideways onto the Charging Station
        new RunCommand(AutoDriveRunnable, driveSubsystem),
        // Once the runnable breaks, it sets the wheels to X position to keep the robot stable
        new InstantCommand(() -> driveSubsystem.setX(), driveSubsystem)
      )
    );

  }
}
