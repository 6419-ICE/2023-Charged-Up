package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

public class HandleArm extends CommandBase{
   
    HashMap<Integer, Double> positions = new HashMap<Integer, Double>();
    int position = 0;
    Arm m_Arm;
    public HandleArm(Arm arm) {
        position = 5;
        //Home Position (Inside Robot)
        positions.put(1, 0.0);
        //Pick up Position
        positions.put(2, 170.0);
        //Cone Position
        positions.put(3, -175.0);
        //Cube Position
        positions.put(4, -200.0);
        //Other side Pick up 
        positions.put(5,-265.0);
        //Vertical 115
        m_Arm = arm;
        addRequirements(m_Arm);
    }
    @Override
    public void execute() {
        SmartDashboard.putNumber("ArmPower", m_Arm.GetPower());
        SmartDashboard.putNumber("ArmEncoder", m_Arm.GetEncoderPos());
        SmartDashboard.putNumber("Position", position);
        // TODO Auto-generated method stub
        if (RobotContainer.GetArmExtendButton()) {
            position++;
            
        } else if (RobotContainer.GetArmRetractButton()) {
            position--;
            
        } 
        if (m_Arm.GetEncoderPos() - 3 > positions.get(position) ) {
        m_Arm.MoveUp();
        } else if (m_Arm.GetEncoderPos() + 3 < positions.get(position)) {
        m_Arm.MoveDown();
        }
        else {
            if (m_Arm.GetEncoderPos() > 180) {
            m_Arm.HoldMotor(true);
        } else {
            m_Arm.HoldMotor(false);
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
