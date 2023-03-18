package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BoolSupplierDriveUntilAngle;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveUntilAngle extends SequentialCommandGroup{
    
    public AutoDriveUntilAngle(DriveSubsystem m_DriveSubsystem, BoolSupplierDriveUntilAngle boolSupplier) {
      RunnableAutoDriveUntilAngle AutoDriveRunnable =  new RunnableAutoDriveUntilAngle(m_DriveSubsystem, boolSupplier);
      RunCommand moveTillAngleRunCommand = new RunCommand(
        AutoDriveRunnable,
        m_DriveSubsystem);
    
    
        addCommands(
      Commands.sequence(
        new InstantCommand(() -> AutoDriveRunnable.hasReachedStation = false), 
        moveTillAngleRunCommand.until(boolSupplier).withTimeout(15),
        new RunCommand(() -> m_DriveSubsystem.setX(), m_DriveSubsystem)
      )
      );
    }

    
}
