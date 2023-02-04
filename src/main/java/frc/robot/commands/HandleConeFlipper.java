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
    DigitalInput testHall = new DigitalInput(8);
    private final ConeFlipper m_ConeFlipper;
    // Constructor, used to pass in The ConeFlipper object to be used
    public HandleConeFlipper(ConeFlipper coneFlipper) {
        m_ConeFlipper = coneFlipper;
        //Makes the ConeFlipper object required by the command
        addRequirements(m_ConeFlipper);
    }

    @Override
    public void end(boolean interrupted) {
<<<<<<< Updated upstream
        //Stops motor once command ends
=======
        down = false;
        up = false;
>>>>>>> Stashed changes
        coneFlipper.StopMotor();
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
//Loops while command is running 
    @Override
    public void execute() {
<<<<<<< Updated upstream
        //checks for button press
        if (RobotContainer.GetConeFlipperUpButton()) {
            //sets the motor to move up
            coneFlipper.MoveUp();
            // ConeFlipper.set(ControlMode.PercentOutput, -0.5);
=======
        if (testHall.get()) {
            pos = false;
        } else {
            pos = true;
        }
        System.out.println(testHall.get() + " POS: " + pos);
        if (RobotContainer.GetConeFlipperUpButton()) {
            up = true; //coneFlipper.MoveUp();
           // System.out.print( new BigDecimal(RobotContainer.GetFlipperPos()).setScale(3, RoundingMode.HALF_UP).stripTrailingZeros().toPlainString() + "-");
           
            testingVar = true;
           // ConeFlipper.set(ControlMode.PercentOutput, -0.5);
>>>>>>> Stashed changes
      } 
      //checks for button press
     else if (RobotContainer.GetConeFlipperDownButton()) {
<<<<<<< Updated upstream
        //sets motor to move down
        coneFlipper.MoveDown();
        //ConeFlipper.set(VictorSPXControlMode.PercentOutput, 0.5);   
      } else {
        //Stops when no button pressed
        coneFlipper.StopMotor();
      }
    //makes up have priority (if both are pressed the motor goes up)
=======
        //System.out.print(  "-" + new BigDecimal(RobotContainer.GetFlipperPos()).setScale(3, RoundingMode.HALF_UP).stripTrailingZeros().toPlainString() );
        down = true;//coneFlipper.MoveDown();
        testingVar = true;
     }
        //ConeFlipper.set(VictorSPXControlMode.PercentOutput, 0.5);   
    //   } else {
    //     if (testingVar) {
    //     System.out.println(System.lineSeparator() + "---");
    //     testingVar = false;
    //     }
    //     coneFlipper.StopMotor();
    //   }
      if (up) {
        System.out.println(testHall.get() + " POS (in up): " + pos);
        if (pos) {
            RobotContainer.GetFlipperEncoder().reset();
            coneFlipper.StopMotor();
            up = false;
            System.out.println("pos = true");
        // if (!(RobotContainer.GetFlipperEncoder().getDistance() < 0)) {
            
        //     System.out.println(RobotContainer.GetFlipperEncoder().getDistance() + "- It Stopped :)");
        //     up = false;
        // } else {
        //     coneFlipper.MoveUp();
        //     System.out.print(RobotContainer.GetFlipperEncoder().getDistance());
            
        // }
    } else if (!pos) {
        System.out.println("pos = false");
        // if (!(RobotContainer.GetFlipperEncoder().getDistance() < 0.275)) {
            
        //     System.out.println(RobotContainer.GetFlipperEncoder().getDistance() + "- It Stopped :)");
        //     up = false;
        // } else {
        //     coneFlipper.MoveUp();
        //     System.out.print(RobotContainer.GetFlipperEncoder().getDistance());
            
        // }
        coneFlipper.MoveUp();
    }
      } else if (down) {
    //     if (pos = false) {
    //     if (!(RobotContainer.GetFlipperEncoder().getDistance() > 0)) {
    //         System.out.println("- It Stopped :)");
    //         down = false;
    //         System.out.println(RobotContainer.GetFlipperEncoder().getDistance());
    //     } else {
    //         System.out.println(RobotContainer.GetFlipperEncoder().getDistance() + "- It Stopped :)");
    //         coneFlipper.MoveDown();
            
    //     }
    // } else {
       if (!(RobotContainer.GetFlipperEncoder().getDistance() > -0.275)) {
            System.out.println("- It Stopped :)");
            down = false;
            System.out.println(RobotContainer.GetFlipperEncoder().getDistance());
        } else {
            System.out.println(RobotContainer.GetFlipperEncoder().getDistance() + "- It Stopped :)");
            coneFlipper.MoveDown();
            
        }
    
    } else {
        coneFlipper.StopMotor();
    }
>>>>>>> Stashed changes
      if (RobotContainer.GetConeFlipperDownButton() && RobotContainer.GetConeFlipperUpButton()) {
        coneFlipper.MoveUp();
      }
    
        // TODO Auto-generated method stub 
        super.execute();
    }

    @Override
    public void initialize() {
       
       
        RobotContainer.GetFlipperEncoder().reset();
        testingVar = false;
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
    
}
