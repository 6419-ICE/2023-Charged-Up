// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmWithPIDAndMotionProfile;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.GrabberWithPIDAndMotionProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

public class GrabberWithPIDProfiledCommand extends ProfiledPIDCommand {
  /** Creates a new RunGrabber. */
  GrabberWithPIDAndMotionProfile m_grabber; 
  //GrabberWithPIDAndMotionProfile m_grabber; 
  public GrabberWithPIDProfiledCommand(GrabberWithPIDAndMotionProfile m_grabber, double goal) 
  {
    
    //super(m_grabber.getController(), m_grabber.getMeasurement(), m_grabber.getController().getGoal(), (m_grabber.getController().calculate(m_grabber.getMeasurement()))
    super(m_grabber.getController(), 
    m_grabber :: getMeasurement, 
    goal,
    (output, setpoint) -> m_grabber.useOutput(output, setpoint), 
    m_grabber);
   
    // Use addRequirements() here to declare subsystem dependencies.
  }



  @Override
  public boolean isFinished() {
    return getController().atGoal();
    // check voltage or check distance within +_ 5 degrees? 

  }
}
