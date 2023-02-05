package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Grabber;

public class HandleGrabber extends CommandBase{
    Grabber m_grabber;
    boolean Open = false;
    boolean Close = false;
    public HandleGrabber() {
    }
    public HandleGrabber(Grabber grabber) {
        m_grabber = grabber;
        addRequirements(m_grabber);
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        if (RobotContainer.GetGrabberCloseButton()) {
       Close = true;
        } else if (RobotContainer.GetGrabberOpenButton()) {
        Open = true;
        } 
        // } else {
        //     m_grabber.StopMotor();
        // }
        //close
        if (Close) {
        if (m_grabber.GetEncoderPos() <= 0) {
            m_grabber.StopMotor();
            Close = false;
        } else {
            m_grabber.CloseGrabber();
        }
    }
        //open 
         else if (Open) {
            if (m_grabber.GetEncoderPos() >=75) {
                m_grabber.StopMotor();
                Open = false;
            } else {
                m_grabber.OpenGrabber();

            }
    }
       
    
        super.execute();
}

    @Override
    public void initialize() {
        
        // TODO Auto-generated method stub
        super.initialize();
    }
    

}
