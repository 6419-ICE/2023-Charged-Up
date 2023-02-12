package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConeFlipper extends SubsystemBase{
    private WPI_VictorSPX ConeFlipperMotor = new WPI_VictorSPX(10);
    private double ConeFlipperMaxPower = 0.5;
    private double ConeFlipperMinPower = -0.5;
    public ConeFlipper() {
        
    }
    
    public void MoveUp() {
        // Sets motor positive, making the Cone Flipper move up
        ConeFlipperMotor.set(VictorSPXControlMode.PercentOutput, ConeFlipperMaxPower);
    }
    public void MoveDown() {
        // Sets motor negative, making the Cone Flipper move down
        ConeFlipperMotor.set(VictorSPXControlMode.PercentOutput, ConeFlipperMinPower);
    }
    public void StopMotor() {
        // Stops motor 
        ConeFlipperMotor.set(VictorSPXControlMode.PercentOutput, 0);
    }
//      if (m_driverController.getAButtonPressed()) {
//         ConeFlipper.set(ControlMode.PercentOutput, -0.5);
//   } else if (m_driverController.getAButtonReleased()) {
//     ConeFlipper.set(VictorSPXControlMode.Position, 0);
//   } 
//   if (m_driverController.getBButtonPressed()) {
//     
    
// } else if (m_driverController.getBButtonReleased()) {
// ConeFlipper.set(VictorSPXControlMode.Position, 0);

// } 
    
}
