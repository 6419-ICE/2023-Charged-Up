package frc.robot.commands;

import java.math.BigDecimal;
import java.math.RoundingMode;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ConeFlipper;

public class HandleConeFlipper extends CommandBase{
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    ConeFlipper coneFlipper = new ConeFlipper();
    boolean testingVar; //Used for Testing Encoder (Should be removed when encoder works)
    boolean down = false;
    boolean up = false;
    boolean pos = true;
   
    private final ConeFlipper m_ConeFlipper;
    // Constructor, used to pass in The ConeFlipper object to be used
    public HandleConeFlipper(ConeFlipper coneFlipper) {
        m_ConeFlipper = coneFlipper;
        //Makes the ConeFlipper object required by the command
        addRequirements(m_ConeFlipper);
    }

    @Override
    public void end(boolean interrupted) {
        down = false;
        up = false;
        coneFlipper.StopMotor();
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
//Loops while command is running 
    @Override
    public void execute() {
        
         //checks for button press
        if (RobotContainer.GetConeFlipperUpButton()) {
            
            coneFlipper.MoveUp();
          
           
           
      } 
      //checks for button press
     else if (RobotContainer.GetConeFlipperDownButton()) {
        
        coneFlipper.MoveDown();
       
     }
        
        else {
        // if (testingVar) {
        // System.out.println(System.lineSeparator() + "---");
        //  testingVar = false;
        // }
         coneFlipper.StopMotor();
       }
     
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
