package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Grabber extends SubsystemBase {

    private TalonFX GrabberMotor = new TalonFX(Constants.DriveConstants.GrabberID);
    private static final double MaxSpeed = 0.05;
    private static final double MinSpeed = -0.05;
    public Grabber() {
       //GrabberMotor.configSelectedFeedbackSensor(GrabberMotor.getSelectedSensorPosition().);
    }
    public void OpenGrabber() {
        GrabberMotor.set(TalonFXControlMode.PercentOutput,MaxSpeed);
        //TalonFXSensorCollection sc =  GrabberMotor.getSensorCollection();
        //System.out.println("Sesor Colection out"+ sc.getIntegratedSensorPosition());
        //System.out.println(GrabberMotor.getSelectedSensorPosition() / Constants.ClawUnitsPerDegree);
    }
    public void CloseGrabber() {
        GrabberMotor.set(TalonFXControlMode.PercentOutput,MinSpeed);
        //System.out.println(GrabberMotor.getSelectedSensorPosition() / Constants.ClawUnitsPerDegree);
    }
    public double GetEncoderPos() {
        return GrabberMotor.getSensorCollection().getIntegratedSensorPosition() / Constants.GrabberConstantsForPIDAndMotionProfile.GrabberUnitsPerDegree;
    }
    public void StopMotor() {
        GrabberMotor.set(ControlMode.PercentOutput, 0);
        //System.out.println(GrabberMotor.getSelectedSensorPosition() / Constants.ClawUnitsPerDegree);
        
    }
}
