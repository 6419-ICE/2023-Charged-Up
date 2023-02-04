package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Grabber;

public class handleGrabber extends CommandBase{
    Grabber m_grabber;
    public handleGrabber() {
    }
    public handleGrabber(Grabber grabber) {
        m_grabber = grabber;
        addRequirements(m_grabber);
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        if (RobotContainer.GetGrabberCloseButton()) {
            m_grabber.CloseGrabber();
        } else if (RobotContainer.GetGrabberOpenButton()) {
            m_grabber.OpenGrabber();
        } else {
            m_grabber.StopMotor();
        }
        super.execute();
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }
    
}
