// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.TrajectoryPaths;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPickForTwoCubes extends SequentialCommandGroup {
  /** Creates a new Autonomous Program. */

  public AutoPickForTwoCubes(DriveSubsystem driveSubsystem, ArmWithPIDAndMotionProfile m_arm, GrabberWithPIDAndMotionProfile m_grabber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.sequence(
        new GrabberWithPIDProfiledCommand(m_grabber, 0).withTimeout(1),
        Commands.parallel(
          new ArmWithPIDProfiledCommand(m_arm, Math.toRadians(170)).withTimeout(5),
          Commands.sequence(
            new WaitCommand(3),
            new GrabberWithPIDProfiledCommand(m_grabber, Math.toRadians(90)).withTimeout(2)
          )
        ),
        Commands.parallel(
          new TrajectoryCommand(driveSubsystem, TrajectoryPaths.trajectoryAutoForwardToPutArmDown()),
          Commands.sequence(
            new WaitCommand(3),
            new ArmWithPIDProfiledCommand(m_arm, Math.toRadians(260)).withTimeout(4)
            )
        ),
        new TurnToAngleProfiled(180, driveSubsystem),
        new TrajectoryCommand(driveSubsystem, TrajectoryPaths.trajectoryAutoForwardTowardsSecondBlock()),
        new GrabberWithPIDProfiledCommand(m_grabber, Math.toRadians(0)).withTimeout(2)
        
        //new ArmToDropOffCommand(m_arm)
        //new WaitCommand(2),
        //new openGrabber(m_grabber)
       /* Commands.parallel(
        new moveArmToGround(m_arm),
        
        new closeGrabber(m_grabber),
        new WaitCommand(2),
        Commands.parallel(
        new MoveBothArmAndGrabberRetract(m_arm, m_grabber),
        new TrajectoryCommand(driveSubsystem, TrajectoryPaths.trajectoryAutoBackTowardsDropOffOfSecondBlock())
        ),
        new ArmToDropOffCommand(m_arm),
        new WaitCommand(2),
        new openGrabber(m_grabber)

         */

      )
    );

  }
}
