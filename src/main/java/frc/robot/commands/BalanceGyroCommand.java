package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.subsystems.DriveSubsystem;

// This BalanceGyroCommand class extends SwerveControllerCommand from the WPI class
// This class should be called when you need the yaw of the x-axis and y-axis to both be zero, i.e. engaged on the charging station
public class BalanceGyroCommand extends TrajectoryCommand {

  private DriveSubsystem m_driveSubSystem;
  private double y_angle = 0;
  private double x_angle = 0;
  final private double length = 1;
 

  public BalanceGyroCommand(DriveSubsystem driveSubSystem) {
       // I shamelessly copy this code from Adrian
    // Adrian is the god of sourcing code
    super(driveSubSystem, new Trajectory());
       
    this.m_driveSubSystem = driveSubSystem;
  }
    
      // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  public double YAngle() {
    return  y_angle;
  }

  public double XAngle() {
    return x_angle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Detects and Saves the angle at that current point and then adjusts position to balance
    m_driveSubSystem.drive(length*x_angle, length*y_angle, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops the robot
    super.end(interrupted);
    m_driveSubSystem.drive(0, 0, 0, false);
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }

  // Formerly useful code
  /*public Trajectory chooseDirection(double angleX, double angleY) {
    int xDirection;
    int yDirection;
    if (angleX > 0.25) {
      xDirection = 1;
    } else if (angleX >= -0.25) {
      xDirection = 0;
    } else {
      xDirection = -1;
    }
    if (angleY > 0.25) {
      yDirection = 1;
    } else if (angleY > -0.25) {
      yDirection = 0;
    } else {
      yDirection = -1;
    }
    return TrajectoryPaths.balanceOnSurface(yDirection, xDirection, angleY, angleX);
  }*/
}
