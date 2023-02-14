package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final double MaxValue = 0.1;
    private final double MinValue = -0.1;
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
        return ArmMotor.getSensorCollection().getIntegratedSensorPosition() / 1024;
    }
     public void StopMotor() {
        //System.out.println(ArmMotor.getSelectedSensorPosition());
        ArmMotor.set(ControlMode.PercentOutput, 0);
    }
    public void HoldMotor(boolean Negative) {
        double holdValue = 0;
        if (Negative) { holdValue = -0.07;}
        else { holdValue = 0.07;}
        
        ArmMotor.set(ControlMode.PercentOutput, holdValue);
    }
    public double GetPower() {
        return ArmMotor.getMotorOutputPercent();
    }
}
