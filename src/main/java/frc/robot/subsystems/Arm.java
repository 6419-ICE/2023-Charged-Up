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
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    private final double aP = 0, aI = 0, aD = 0;
    private final double aF = 0;
    
    PIDController PID = new PIDController(aP, aI, aD);

    private TalonFX ArmMotor = new TalonFX(Constants.DriveConstants.ArmID);
    double velocity = 0;

    public void pickUp() {
        velocity = PID.calculate(ArmMotor.getSensorCollection().getIntegratedSensorPosition(), Constants.ArmConstantsForPIDAndMotionProfile.homePosition) + aF;
        ArmMotor.set(ControlMode.Velocity, velocity);
       // System.out.println(ArmMotor.getSelectedSensorPosition());
    }

    public void dropOff() {
        velocity = PID.calculate(ArmMotor.getSensorCollection().getIntegratedSensorPosition(), Constants.ArmConstantsForPIDAndMotionProfile.dropPosition) + aF;
        ArmMotor.set(ControlMode.Velocity, velocity);
       // System.out.println(ArmMotor.getSelectedSensorPosition());
    }

    public void inRobot() {
        velocity = PID.calculate(ArmMotor.getSensorCollection().getIntegratedSensorPosition(), Constants.ArmConstantsForPIDAndMotionProfile.homePosition) + aF;
        ArmMotor.set(ControlMode.Velocity, velocity);
       // System.out.println(ArmMotor.getSelectedSensorPosition());
    }

    public double GetEncoderPos() {
        return ArmMotor.getSensorCollection().getIntegratedSensorPosition();
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

    public double aP() {
        return aP;
    }
    public double aI() {
        return aI;
    }
    public double aD() {
        return aD;
    }
    public double aF() {
        return aF;
    }
}
