package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    CANSparkMax intakeMotor = new CANSparkMax(Constants.DriveConstants.GrabberID,MotorType.kBrushless);
    public void setSpeed(double val) {
        intakeMotor.set(val);
    
    }
    public double getEncoderVal() {
        return intakeMotor.getEncoder().getPosition();
    }
}