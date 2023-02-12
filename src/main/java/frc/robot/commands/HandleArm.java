package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;

public class HandleArm extends CommandBase{
    Arm m_Arm;
    public HandleArm(Arm arm) {
        m_Arm = arm;
        addRequirements(m_Arm);
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        if (RobotContainer.GetArmExtendButton()) {
            m_Arm.MoveUp();
        } else if (RobotContainer.GetArmRetractButton()) {
            m_Arm.MoveDown();
        } else {
            m_Arm.StopMotor();
        }

        super.execute();
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }
    
}
