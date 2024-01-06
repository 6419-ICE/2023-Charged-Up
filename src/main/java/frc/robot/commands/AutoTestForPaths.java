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
public class AutoTestForPaths extends SequentialCommandGroup {
  /** Creates a new Autonomous Program. */

  public AutoTestForPaths(DriveSubsystem driveSubsystem) {
  
    addCommands(
      
      Commands.sequence(
        new TrajectoryCommand(driveSubsystem, TrajectoryPaths.GoForwardToTurnFast()),
        new TurnToAngleProfiled(10, driveSubsystem).withTimeout(0.5),
        new TurnToAngleProfiled(179.999, driveSubsystem).withTimeout(3),
        new TrajectoryCommand(driveSubsystem, TrajectoryPaths.GoBackwardsTowardsBlockFast()),
        new TrajectoryCommand(driveSubsystem, TrajectoryPaths.GoForwardToTurnOnWayBackFast()),
        new TurnToAngleProfiled(160, driveSubsystem).withTimeout(0.5),
        new TurnToAngleProfiled(0, driveSubsystem).withTimeout(3),
        new TrajectoryCommand(driveSubsystem, TrajectoryPaths.GoBackwardsToDropBlockFast())

       // new WaitCommand(2),
       // new TurnToAngleProfiled(-179.999, driveSubsystem).withTimeout(3), 
       // new TrajectoryCommand(driveSubsystem, TrajectoryPaths.trajectoryAutoForwardTowardsSecondBlock()),
       // new TrajectoryCommand(driveSubsystem, TrajectoryPaths.trajectoryAutoForwardBackFromSecondBlock()),
       // new TurnToAngleProfiled(0, driveSubsystem).withTimeout(3)
        
        )
    );

  }
}
