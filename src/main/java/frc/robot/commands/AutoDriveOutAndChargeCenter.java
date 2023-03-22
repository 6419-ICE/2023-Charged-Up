package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryPaths;
import frc.robot.subsystems.ArmWithPIDAndMotionProfile;
import frc.robot.subsystems.BoolSupplierDriveUntilAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GrabberWithPIDAndMotionProfile;

public class AutoDriveOutAndChargeCenter extends SequentialCommandGroup{
    
    public AutoDriveOutAndChargeCenter(DriveSubsystem m_DriveSubsystem, ArmWithPIDAndMotionProfile m_arm, GrabberWithPIDAndMotionProfile m_grabber, BoolSupplierDriveUntilAngle boolSupplier) {
      RunnableDriveBackwardsUntilAngle AutoDriveRunnable =  new RunnableDriveBackwardsUntilAngle(m_DriveSubsystem, boolSupplier, false);
      RunCommand moveTillAngleRunCommand = new RunCommand(
        AutoDriveRunnable,
        m_DriveSubsystem);
    
    
        addCommands(
      Commands.sequence(
        new ArmToDropOffCommand(m_arm), withTimeout(2),
        new openGrabber(m_grabber), withTimeout(1),
        new MoveBothArmAndGrabberRetract(m_arm, m_grabber), withTimeout(2)//,
        /*new TrajectoryCommand(m_DriveSubsystem, TrajectoryPaths.trajectoryAutoEngageOnChargingStation()),
        new InstantCommand(() -> AutoDriveRunnable.resetRunnable()), 
        moveTillAngleRunCommand.until(boolSupplier).withTimeout(15),
        new InstantCommand(() -> m_DriveSubsystem.setX(), m_DriveSubsystem)*/
      )
      );
    }

    
}
