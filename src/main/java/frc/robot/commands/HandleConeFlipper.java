package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ConeFlipper;

public class HandleConeFlipper extends CommandBase{
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    ConeFlipper coneFlipper = new ConeFlipper();

    private final ConeFlipper m_ConeFlipper;
    public HandleConeFlipper(ConeFlipper coneFlipper) {
        m_ConeFlipper = coneFlipper;
        addRequirements(m_ConeFlipper);
    }

    @Override
    public void end(boolean interrupted) {
        coneFlipper.StopMotor();
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public void execute() {
        if (RobotContainer.GetConeFlipperUpButton()) {
            coneFlipper.MoveUp();
           // ConeFlipper.set(ControlMode.PercentOutput, -0.5);
      } 
     else if (RobotContainer.GetConeFlipperDownButton()) {
        coneFlipper.MoveDown();
        //ConeFlipper.set(VictorSPXControlMode.PercentOutput, 0.5);   
      } else {
        coneFlipper.StopMotor();
      }
    
      if (RobotContainer.GetConeFlipperDownButton() && RobotContainer.GetConeFlipperUpButton()) {
        coneFlipper.MoveUp();
      }
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
    
}
