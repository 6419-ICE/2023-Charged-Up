package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.BoolSupplierDriveUntilAngle;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;

public class RunnableAutoDriveUntilAngle implements Runnable {
    DriveSubsystem m_DriveSubsystem;
    public Boolean hasReachedStation = false;
    Boolean hasOvershot = false;
    BoolSupplierDriveUntilAngle boolSupplier;
    int angleValue = 1;
    int yMultiplier = 0;
    int xMultiplier = 0;
    double previousAngle;
    public RunnableAutoDriveUntilAngle(DriveSubsystem m_DriveSubsystem, BoolSupplierDriveUntilAngle boolSupplier, Boolean isSideways) {
        m_DriveSubsystem.ResetGyro();
        this.m_DriveSubsystem = m_DriveSubsystem;
        this.boolSupplier = boolSupplier;
        hasReachedStation = false;
        if (isSideways) {
            angleValue = 0;
            xMultiplier = 0;
            yMultiplier = 1;
        } else {
            angleValue = 1;
            xMultiplier = 1;
            yMultiplier = 0;
        }
       
    }

    @Override
    public void run() {

        
        // TODO Auto-generated method stub

        SmartDashboard.putNumber("xAngle", m_DriveSubsystem.getAngles()[0]);
        SmartDashboard.putNumber("yAngle", m_DriveSubsystem.getAngles()[1]);
        SmartDashboard.putNumber("zAngle", m_DriveSubsystem.getAngles()[2]);
        if ((m_DriveSubsystem.getAngles()[angleValue] < -11) || (hasReachedStation)) {
            //Moves on station
            hasReachedStation = true;
            System.out.println("Keep Driving");
            m_DriveSubsystem.drive(xMultiplier * 0.1, yMultiplier * 0.1, 0, true);
            //Checks if robot is falling
            if (((m_DriveSubsystem.getAngles()[angleValue] > previousAngle + 0.07)) && 
            ((m_DriveSubsystem.getAngles()[angleValue] >  -10))/*MathUtil.applyDeadband(m_DriveSubsystem.getAngles()[1], 3) == 0*/) {
                //stopped
                    m_DriveSubsystem.drive(0, 0, 0, true);
                    m_DriveSubsystem.setX();
                    try {
                        Thread.sleep(500); // Check if this is enough time
                    } catch (InterruptedException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                    }
                    // if (m_DriveSubsystem.getAngles()[1] > 5){
                    //     //Correction
                    //          m_DriveSubsystem.drive(-0.1, 0, 0, true);
                    // }
                     if (MathUtil.applyDeadband(m_DriveSubsystem.getAngles()[angleValue], 3) == 0 ) {boolSupplier.SetValue(true);}
                    
                    System.out.println("Finished");
            } //Re-Add after this if statement ^ ?
            
        } else if (!hasReachedStation) {
            System.out.println("Still Driving");
            m_DriveSubsystem.drive(xMultiplier * 0.20, yMultiplier * 0.20, 0, true);
        }
        // else {
        // System.out.println("Error");
        // m_DriveSubsystem.drive(0, 0, 0, true);
        // }
            previousAngle = m_DriveSubsystem.getAngles()[angleValue];
    }
    
}
