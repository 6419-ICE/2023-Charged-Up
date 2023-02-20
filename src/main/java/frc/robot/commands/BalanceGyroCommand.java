package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

// This BalanceGyroCommand class extends SwerveControllerCommand from the WPI class
// This class should be called when you need the yaw of the x-axis and y-axis to both be zero, i.e. engaged on the charging station
public class BalanceGyroCommand extends SwerveControllerCommand {

  private ADIS16470_IMU imu;
  private DriveSubsystem m_driveSubSystem;
  private Trajectory m_Trajectory;
  private double y_angle;
  private double x_angle;
  ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  public BalanceGyroCommand(DriveSubsystem driveSubSystem, Trajectory trajectory, ADIS16470_IMU ADIS16470) {
    // I shamelessly copy this code from Adrian
    super(trajectory,
        driveSubSystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),
        driveSubSystem::setModuleStates,
        driveSubSystem);
    this.imu = ADIS16470;
    this.m_driveSubSystem = driveSubSystem;
    this.m_Trajectory = trajectory;
    this.thetaController.enableContinuousInput(-Math.PI,Math.PI);
    addRequirements(driveSubSystem);
  }
    
      // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    imu.setYawAxis(IMUAxis.kY);
    this.y_angle = imu.getAngle();
    imu.setYawAxis(IMUAxis.kX);
    this.x_angle = imu.getAngle();
    if ((this.x_angle > 0) && (this.y_angle > 0)) {

    } else if ((this.x_angle < 0) && (this.y_angle > 0)) {

    } else if ((this.x_angle == 0) && (this.y_angle > 0)) {

    } else if ((this.x_angle > 0) && (this.y_angle == 0)) { 

    } else if ((this.x_angle < 0) && (this.y_angle == 0)) { 
      
    } else if ((this.x_angle > 0) && (this.y_angle < 0)) { 
      
    } else if ((this.x_angle == 0) && (this.y_angle < 0)) { 
      
    } else if ((this.x_angle < 0) && (this.y_angle < 0)) { 
      
    } else {
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_driveSubSystem.drive(0, 0, 0, false);
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}
