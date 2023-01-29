package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;


// This is the TrajectoryCommand Class, which extends the swerveControllerComandClass from wplib. This class is a command.
// You should call this command in automous(see the autoDriveOutOfCommunity), in conjunction with other commands. 
public class TrajectoryCommand extends SwerveControllerCommand{

    private DriveSubsystem m_driveSubSystem;
    private Trajectory m_Trajectory;
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
   

        
    public TrajectoryCommand(DriveSubsystem driveSubSystem, Trajectory trajectory)
    {
        //thetaController.enableContinuousInput(-Math.PI, Math.PI);
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
        this.m_Trajectory = trajectory; 
        this.m_driveSubSystem = driveSubSystem;
        //this.the
        this.thetaController.enableContinuousInput(-Math.PI,Math.PI);
        addRequirements(driveSubSystem);
    }

    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_driveSubSystem.setMaxMotorSpeed(Constants.DrivetrainConstants.speedLmt);
    //m_drive.resetHeading();
    m_driveSubSystem.resetOdometry(m_Trajectory.getInitialPose());
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("execute");
    super.execute();
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
