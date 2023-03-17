package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    m_driveSubSystem.ResetGyro();
    
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
    SmartDashboard.putNumber("xAngle", m_driveSubSystem.getAngles()[0]);
    SmartDashboard.putNumber("yAngle", m_driveSubSystem.getAngles()[1]);
    SmartDashboard.putNumber("zAngle", m_driveSubSystem.getAngles()[2]);
    m_driveSubSystem.drive(0.8, 0, 0, true);
    //new WaitCommand(3);
    // if (!(anglesList[1] > Math.toRadians(5))) {
    // // && (this.m_isForward) || ((this. x_angle < -5) && (!this.m_isForward)) || ((this.y_angle > 5) && (this.m_isForward)) || ((this.y_angle < -5) && (!this.m_isForward))) {
    //   //m_driveSubSystem.setModuleStates(new SwerveModuleState[] {new SwerveModuleState(5.0, new Rotation2d(0)),new SwerveModuleState(5.0, new Rotation2d(0)),new SwerveModuleState(5.0, new Rotation2d(0)),new SwerveModuleState(5.0, new Rotation2d(0))});
      
    // } else {
    //   end(false);
    // }
    super.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_driveSubSystem.setModuleStates(new SwerveModuleState[] {new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState()});
   m_driveSubSystem.drive(0,0,0,true);
    super.end(interrupted);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }

}
