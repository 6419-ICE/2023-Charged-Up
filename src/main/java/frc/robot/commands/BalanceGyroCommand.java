package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.Constants.DriveConstants;
import frc.robot.TrajectoryPaths;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

// This BalanceGyroCommand class extends SwerveControllerCommand from the WPI class
// This class should be called when you need the yaw of the x-axis and y-axis to both be zero, i.e. engaged on the charging station
public class BalanceGyroCommand extends SwerveControllerCommand {

  private ADIS16470_IMU imu;
  private DriveSubsystem m_driveSubSystem;
  private double y_angle = 0;
  private double x_angle = 0;
  final private double length = 1;
  static TrajectoryConfig config = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);
  ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  // Empty Trajectory
  private static Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(0,0)), new Pose2d((Units.inchesToMeters(0)), 0, new Rotation2d(0)), config);
 

  public BalanceGyroCommand(DriveSubsystem driveSubSystem) {
       // I shamelessly copy this code from Adrian
    // Adrian is the god of sourcing code
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
    this.imu = driveSubSystem.m_gyro;
    this.m_driveSubSystem = driveSubSystem;
    this.thetaController.enableContinuousInput(-Math.PI,Math.PI);
        addRequirements(driveSubSystem);
  }
    
      // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  public double YAngle() {
    return  y_angle;
  }

  public double XAngle() {
    return x_angle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Detects and Saves the angle at that current point and then adjusts position to balance
    imu.setYawAxis(IMUAxis.kY);
    this.y_angle = imu.getAngle();
    imu.setYawAxis(IMUAxis.kX);
    this.x_angle = imu.getAngle();
    m_driveSubSystem.drive(length*x_angle, length*y_angle, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops the robot
    super.end(interrupted);
    imu.setYawAxis(IMUAxis.kZ);
    m_driveSubSystem.drive(0, 0, 0, false);
  } 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.isFinished();
  }

  // Formerly useful code
  /*public Trajectory chooseDirection(double angleX, double angleY) {
    int xDirection;
    int yDirection;
    if (angleX > 0.25) {
      xDirection = 1;
    } else if (angleX >= -0.25) {
      xDirection = 0;
    } else {
      xDirection = -1;
    }
    if (angleY > 0.25) {
      yDirection = 1;
    } else if (angleY > -0.25) {
      yDirection = 0;
    } else {
      yDirection = -1;
    }
    return TrajectoryPaths.balanceOnSurface(yDirection, xDirection, angleY, angleX);
  }*/
}
