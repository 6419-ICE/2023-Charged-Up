package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class BalanceOnChargeStation extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  private DriveSubsystem drive; 
  public BalanceOnChargeStation(DriveSubsystem drive) {
    super(
        new ProfiledPIDController(
            0.05,
            ModuleConstants.kTurningI,
            ModuleConstants.kTurningD,
            new TrapezoidProfile.Constraints(
                0.1,
                0.1)), //Reminder, make sure to change this variable/make a new one if the program works. 
        // Close loop on heading
        drive::getHeadingForBalancing,
        // Set reference to target
        0,
        // Pipe output to turn robot
        (output, setpoint) -> drive.balance(output,0,0, true),
        // Require the drive
        drive);
        this.drive = drive; 

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        //.setTolerance(ModuleConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
        .setTolerance(1, 1);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
   // End when the controller is at the reference
   boolean finished = false; 
   if(Math.abs(drive.getHeadingForBalancing()-getController().getGoal().position) < getController().getPositionTolerance())
   {
     finished = true;

   }
   else
   {
     finished = false;
   }
   SmartDashboard.putBoolean("balanceIsFinish", finished); 
   return finished; 
  }
}