// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmWithPIDAndMotionProfile;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.GrabberWithPIDAndMotionProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HandleArmWithPIDAndMotionProfile extends CommandBase {
  /** Creates a new RunGrabber. */
  ArmWithPIDAndMotionProfile m_arm; 
  GrabberWithPIDAndMotionProfile m_grabber; 
  public HandleArmWithPIDAndMotionProfile(ArmWithPIDAndMotionProfile m_arm) 
  {
    this.m_arm = m_arm; 
    //this.m_grabber = m_grabber; 
    addRequirements(this.m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }



   // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  /*   if(RobotContainer.GetArmRetractButton())
    {
      //m_grabber.CloseFully();
      m_arm.Home();
    } */

  
    if(RobotContainer.GetArmExtendButton())
    {
      m_arm.Top();
    }
    if(RobotContainer.GetArmGroundButton())
    {
      m_arm.Ground();
    }

  
    SmartDashboard.putNumber("PositionInDegreesArm", Math.toDegrees(m_arm.getMeasurement()));
  }
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
