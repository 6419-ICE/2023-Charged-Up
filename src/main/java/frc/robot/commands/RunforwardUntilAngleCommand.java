package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;


// This is a command to go forward practically infinitely until it reaches a certain angle
public class RunforwardUntilAngleCommand extends TrajectoryCommand{
  
  private DriveSubsystem m_driveSubSystem;
  private Trajectory m_Trajectory;
  private boolean m_isForward;
  private double x_angle;
  private ADIS16470_IMU imu;

    public RunforwardUntilAngleCommand(DriveSubsystem driveSubSystem, Trajectory trajectory, boolean isForward)
    {
      super(driveSubSystem, trajectory);
      m_isForward = isForward;
      m_driveSubSystem = driveSubSystem;
      m_Trajectory = trajectory;
    }

    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubSystem.resetOdometry(m_Trajectory.getInitialPose());
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("execute");
    super.execute();
    imu.setYawAxis(IMUAxis.kX);
    this.x_angle = imu.getAngle();
    SmartDashboard.putNumber("xAngle", x_angle);
    if (((this.x_angle > 5) && (this.m_isForward)) || ((this.x_angle < -5) && (!this.m_isForward))) {
      end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    imu.setYawAxis(IMUAxis.kZ);
    m_driveSubSystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }

}
