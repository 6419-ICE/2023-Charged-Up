package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {

    private TalonFX GrabberMotor = new TalonFX(Constants.DriveConstants.GrabberID);
    private static final double MaxSpeed = 0.05;
    private static final double MinSpeed = -0.05;
    public Grabber() {
       
    }
    public void OpenGrabber() {
        GrabberMotor.set(TalonFXControlMode.PercentOutput,MaxSpeed);
        System.out.println(GrabberMotor.getSelectedSensorPosition() / Constants.ClawUnitsPerDegree);
    }
    public void CloseGrabber() {
        GrabberMotor.set(TalonFXControlMode.PercentOutput,MinSpeed);
        System.out.println(GrabberMotor.getSelectedSensorPosition() / Constants.ClawUnitsPerDegree);
    }
    public double GetEncoderPos() {
        return GrabberMotor.getSelectedSensorPosition() / 768;
    }
    public void StopMotor() {
        GrabberMotor.set(ControlMode.PercentOutput, 0);
        System.out.println(GrabberMotor.getSelectedSensorPosition() / Constants.ClawUnitsPerDegree);
        
    }
}
