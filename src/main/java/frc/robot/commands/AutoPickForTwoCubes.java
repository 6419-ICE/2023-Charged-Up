// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.TrajectoryPaths;
import frc.robot.subsystems.ArmWithPIDAndMotionProfile;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GrabberWithPIDAndMotionProfile;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPickForTwoCubes extends SequentialCommandGroup {
  /** Creates a new Autonomous Program. */

  public AutoPickForTwoCubes(DriveSubsystem driveSubsystem, ArmWithPIDAndMotionProfile m_arm, GrabberWithPIDAndMotionProfile m_grabber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    driveSubsystem.resetEncoders();
    SmartDashboard.putNumberArray("before", driveSubsystem.getEncoderPositions());
    addCommands(
      Commands.sequence (
        new TrajectoryCommand(driveSubsystem, TrajectoryPaths.trajectoryMoveToCube())
        //new ArmToDropOffCommand(m_arm), withTimeout(5)
        
        // new openGrabber(m_grabber), withTimeout(5),
        
        // new moveArmToGround(m_arm), withTimeout(5),
        // new TrajectoryCommand(driveSubsystem, TrajectoryPaths.trajectoryAutoForwardTowardsSecondBlock()), withTimeout(5),
        // new closeGrabber(m_grabber), withTimeout(5),
        
        
        // new MoveBothArmAndGrabberRetract(m_arm, m_grabber), withTimeout(5),
        // new TrajectoryCommand(driveSubsystem, TrajectoryPaths.trajectoryAutoBackTowardsDropOffOfSecondBlock()), withTimeout(5),
        // new ArmToDropOffCommand(m_arm), withTimeout(5),
        // new openGrabber(m_grabber)
    
        
      )
    );
    SmartDashboard.putNumberArray("after", driveSubsystem.getEncoderPositions());
  }
}
