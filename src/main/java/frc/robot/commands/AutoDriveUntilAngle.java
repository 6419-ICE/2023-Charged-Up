package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveUntilAngle extends SequentialCommandGroup{
    public AutoDriveUntilAngle(DriveSubsystem m_DriveSubsystem) {
        addCommands(
      Commands.sequence(
        new RunforwardUntilAngleCommand(m_DriveSubsystem,true)
        //new BalanceGyroCommand(driveSubsystem)
      )
    );
    }
}
