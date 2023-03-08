package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


// This is a command to go forward practically infinitely until it reaches a certain angle
public class RunforwardUntilAngleCommand extends CommandBase {
  
  private DriveSubsystem m_driveSubSystem;
  private boolean m_isForward;
  private double x_angle;
  private double y_angle;
  private double z_angle;

  public RunforwardUntilAngleCommand(DriveSubsystem driveSubSystem, boolean isForward)
  {
    //Constructor that builds the object and stores the direction its going
      m_isForward = isForward;
      m_driveSubSystem = driveSubSystem;
  }

    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubSystem.drive(0.4, 0, 0, true);
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("execute");
    double[] anglesList = m_driveSubSystem.getAngles();
    x_angle = anglesList[0];
    y_angle = anglesList[1];
    z_angle = anglesList[2];
    SmartDashboard.putNumber("xAngle", x_angle);
    SmartDashboard.putNumber("yAngle", y_angle);
    SmartDashboard.putNumber("zAngle", z_angle);
    if /*(((*/(this.x_angle > Math.toRadians(5)) {
    // && (this.m_isForward) || ((this.x_angle < -5) && (!this.m_isForward)) || ((this.y_angle > 5) && (this.m_isForward)) || ((this.y_angle < -5) && (!this.m_isForward))) {
      end(true);
    }
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_driveSubSystem.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }

}
