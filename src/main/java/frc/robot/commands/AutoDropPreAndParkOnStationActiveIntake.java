// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmWithPIDAndMotionProfile;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.TrajectoryPaths;
import frc.robot.commands.IntakeAutoCommand.state;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDropPreAndParkOnStationActiveIntake extends SequentialCommandGroup {
  /** Creates a new Autonomous Program. */

  public AutoDropPreAndParkOnStationActiveIntake(DriveSubsystem driveSubsystem, ArmWithPIDAndMotionProfile m_arm, IntakeSubsystem m_intake) {
  
    addCommands(
      Commands.sequence(
        Commands.parallel(
          new ArmWithPIDProfiledCommand(m_arm, Math.toRadians(Constants.ArmConstantsForPIDAndMotionProfile.dropPosition)).withTimeout(4),
          Commands.sequence(
            new WaitCommand(3),
            new IntakeAutoCommand(m_intake, state.cubeOut).withTimeout(0.5)
            )
          ),
    Commands.parallel(
              new ArmWithPIDProfiledCommand(m_arm, Math.toRadians(Constants.ArmConstantsForPIDAndMotionProfile.homePosition)).withTimeout(3),
              //new GrabberWithPIDProfiledCommand(m_grabber, Math.toRadians(0)).withTimeout(3),  only needed for grabber
              Commands.sequence(
              new WaitCommand(2),
              new TrajectoryCommand(driveSubsystem, TrajectoryPaths.MoveOutOfCommunity())
            )
            ),
            new TrajectoryCommand(driveSubsystem, TrajectoryPaths.BackOnCharge()),
            new RunCommand(() -> driveSubsystem.setX())
            

        )


      );
  

  }
}
