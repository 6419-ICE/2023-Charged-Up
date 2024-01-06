// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.TrajectoryPaths;
import frc.robot.subsystems.ArmWithPIDAndMotionProfile;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GrabberWithPIDAndMotionProfile;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveOutOfCommunityWithWIres extends SequentialCommandGroup {
  /** Creates a new Autonomous Program. */

  public AutoDriveOutOfCommunityWithWIres(DriveSubsystem driveSubsystem, ArmWithPIDAndMotionProfile m_arm, GrabberWithPIDAndMotionProfile m_grabber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.sequence(
        Commands.parallel(
          new ArmWithPIDProfiledCommand(m_arm, Math.toRadians(170)).withTimeout(4),
          Commands.sequence(
            new WaitCommand(2),
            new GrabberWithPIDProfiledCommand(m_grabber, Math.toRadians(100)).withTimeout(1)
            )
          ),
    Commands.parallel(
              new ArmWithPIDProfiledCommand(m_arm, Math.toRadians(1)).withTimeout(3),
              new GrabberWithPIDProfiledCommand(m_grabber, Math.toRadians(0)).withTimeout(3)
            ),
            Commands.parallel(
              new TrajectoryCommand(driveSubsystem, TrajectoryPaths.MoveOutOfCommunityOverWire())
        )
            )
        );
       

  }
}
