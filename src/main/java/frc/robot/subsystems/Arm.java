package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final double MaxValue = 0.05;
    private final double MinValue = -0.05;
    private TalonFX ArmMotor = new TalonFX(Constants.DriveConstants.ArmID);
    public Arm() {
        
    }
    public void MoveUp() {
        ArmMotor.set(ControlMode.PercentOutput, MinValue);
       // System.out.println(ArmMotor.getSelectedSensorPosition());
    }
    public void MoveDown() {
        ArmMotor.set(ControlMode.PercentOutput, MaxValue);
       // System.out.println(ArmMotor.getSelectedSensorPosition());
    }
    public double GetEncoderPos() {
        return ArmMotor.getSelectedSensorPosition();
    }
     public void StopMotor() {
      //  System.out.println(ArmMotor.getSelectedSensorPosition());
        ArmMotor.set(ControlMode.PercentOutput, 0);
    }
}
