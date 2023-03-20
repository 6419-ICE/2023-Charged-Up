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
    // Constructor that builds using a blank trajectory
    super(driveSubSystem, new Trajectory());
    this.m_driveSubSystem = driveSubSystem;
    this.x_angle = driveSubSystem.getAngles()[0];
    addRequirements(driveSubSystem);
  }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Detects and Saves the angle at that current point and then adjusts position to balance
    x_angle = m_driveSubSystem.getAngles()[0];
    y_angle = m_driveSubSystem.getAngles()[1];
    x_angle = angleAdjustor(x_angle, y_angle)[0];
    y_angle = angleAdjustor(x_angle, y_angle)[1];
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

  // Repurposed code that changes the angle value to a zero if it passes a certain threshold. 
  public double[] angleAdjustor(double angleX, double angleY) {
    double adjustedXAngle = angleX;
    double adjustedYAngle = angleY;
    if (!((angleX > 0.25) && (angleX < -0.25))) {
      adjustedXAngle = 0;
    }
    if (!((angleY > 0.25) && (angleY < -0.25))) {
      adjustedYAngle = 0;
    }
    double[] returnArray = {adjustedXAngle, adjustedYAngle};
    return returnArray;
  }
}
