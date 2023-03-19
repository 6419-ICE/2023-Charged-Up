package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryPaths;
import frc.robot.subsystems.BoolSupplierDriveUntilAngle;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveOutAndChargeCenter extends SequentialCommandGroup{
    
    public AutoDriveOutAndChargeCenter(DriveSubsystem m_DriveSubsystem, BoolSupplierDriveUntilAngle boolSupplier) {
      RunnableDriveBackwardsUntilAngle AutoDriveRunnable =  new RunnableDriveBackwardsUntilAngle(m_DriveSubsystem, boolSupplier, false);
      RunCommand moveTillAngleRunCommand = new RunCommand(
        AutoDriveRunnable,
        m_DriveSubsystem);
    
    
        addCommands(
      Commands.sequence(
        new TrajectoryCommand(m_DriveSubsystem, TrajectoryPaths.trajectoryAutoEngageOnChargingStation()),
        new InstantCommand(() -> AutoDriveRunnable.resetRunnable()), 
        moveTillAngleRunCommand.until(boolSupplier).withTimeout(15),
        new InstantCommand(() -> m_DriveSubsystem.setX(), m_DriveSubsystem)
      )
      );
    }

    
}
