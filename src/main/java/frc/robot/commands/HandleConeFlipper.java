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
    // Constructor, used to pass in The ConeFlipper object to be used
    public HandleConeFlipper(ConeFlipper coneFlipper) {
        m_ConeFlipper = coneFlipper;
        //Makes the ConeFlipper object required by the command
        addRequirements(m_ConeFlipper);
    }

    @Override
    public void end(boolean interrupted) {
        //Stops motor once command ends
        coneFlipper.StopMotor();
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
//Loops while command is running 
    @Override
    public void execute() {
        //checks for button press
        if (RobotContainer.GetConeFlipperUpButton()) {
            //sets the motor to move up
            coneFlipper.MoveUp();
            // ConeFlipper.set(ControlMode.PercentOutput, -0.5);
      } 
      //checks for button press
     else if (RobotContainer.GetConeFlipperDownButton()) {
        //sets motor to move down
        coneFlipper.MoveDown();
        //ConeFlipper.set(VictorSPXControlMode.PercentOutput, 0.5);   
      } else {
        //Stops when no button pressed
        coneFlipper.StopMotor();
      }
    //makes up have priority (if both are pressed the motor goes up)
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
