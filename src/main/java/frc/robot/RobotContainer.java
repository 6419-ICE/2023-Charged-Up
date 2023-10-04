// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.GrabberConstantsForPIDAndMotionProfile;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.HandleConeFlipper;
import frc.robot.commands.handleGrabber;
import frc.robot.commands.HandleGrabberWithPIDAndMotionProfile;
import frc.robot.commands.MoveBothArmAndGrabberRetract;
import frc.robot.commands.RunforwardUntilAngleCommand;
import frc.robot.commands.HandleArmWithPIDAndMotionProfile;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ConeFlipper;

import frc.robot.commands.AutoDriveOutAndChargeLeft;
import frc.robot.commands.AutoDriveOutAndChargeRight;
import frc.robot.commands.AutoDriveOutOfCommunity;
import frc.robot.commands.AutoDriveUntilAngle;
import frc.robot.commands.AutoEngageOnChargingStation;
import frc.robot.commands.AutoPickForTwoCubes;
import frc.robot.commands.HandleArm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.GrabberWithPIDAndMotionProfile;
import frc.robot.subsystems.ArmWithPIDAndMotionProfile;
import frc.robot.subsystems.BoolSupplierDriveUntilAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ConeFlipper m_ConeFlipper = new ConeFlipper();
  //private final Grabber m_Grabber = new Grabber();
  private final GrabberWithPIDAndMotionProfile m_GrabberWithPID = new GrabberWithPIDAndMotionProfile(); 
  private final ArmWithPIDAndMotionProfile m_ArmWithPID = new ArmWithPIDAndMotionProfile(); 
  private final Arm m_ArmForEncoderPos = new Arm(); 

  //private final Arm m_Arm = new Arm();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  static Joystick mechanismJoystick = new Joystick(Constants.ButtonBoxID);
  static JoystickButton coneFlipperUpButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.ConeFlipperUp);
  static JoystickButton coneFlipperDownButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.ConeFlipperDown);
  static JoystickButton GrabberOpenButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.GrabberOpen);
  static JoystickButton GrabberCloseButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.GrabberClose);
  private HandleConeFlipper handleConeFlipper = new HandleConeFlipper(m_ConeFlipper);
  //private HandleGrabber handleGrabber = new HandleGrabber(m_Grabber);
  private HandleGrabberWithPIDAndMotionProfile handleGrabberWithPID = new HandleGrabberWithPIDAndMotionProfile(m_GrabberWithPID);
  private HandleArmWithPIDAndMotionProfile handleArmWithPID = new HandleArmWithPIDAndMotionProfile(m_ArmWithPID);
  private MoveBothArmAndGrabberRetract moveBothArmAndGrabber = new MoveBothArmAndGrabberRetract(m_ArmWithPID, m_GrabberWithPID);

private BoolSupplierDriveUntilAngle boolSupplier = new BoolSupplierDriveUntilAngle();
  //private HandleArm handleArm = new HandleArm(m_Arm);
  private static DutyCycleEncoder coneFlipperEncoder = new DutyCycleEncoder(Constants.FlipperEncoderID);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private static SendableChooser<Command> autoChooser;
  public RobotContainer() {
    // Configure the button bindings
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("Auto Drive Out Of Community", new AutoDriveOutOfCommunity(m_robotDrive));
    autoChooser.addOption("Auto Two Cubes", new AutoPickForTwoCubes(m_robotDrive,m_ArmWithPID,m_GrabberWithPID));
    autoChooser.addOption("Auto Engage on Charging Station Center", new AutoEngageOnChargingStation(m_robotDrive));
    autoChooser.addOption("Auto Charge on Charging Station Left", new AutoDriveOutAndChargeLeft(m_robotDrive));
    autoChooser.addOption("Auto Charge on Charging Station Right ", new AutoDriveOutAndChargeRight(m_robotDrive));
    autoChooser.addOption("Auto Run Until Angle", new AutoDriveUntilAngle(m_robotDrive, boolSupplier));
    SmartDashboard.putData("Autonomous", autoChooser);
    
    
    SmartDashboard.putNumber("kP", Constants.ArmConstantsForPIDAndMotionProfile.kP);
    SmartDashboard.putNumber("kI", Constants.ArmConstantsForPIDAndMotionProfile.kI);
    SmartDashboard.putNumber("kD", Constants.ArmConstantsForPIDAndMotionProfile.kD);
    SmartDashboard.putNumber("Arm Encoder Position", m_ArmForEncoderPos.GetEncoderPos());


    

    //Shuffleboard.getTab("Gryo tab").add(m_robotDrive.m_gyro);

    configureButtonBindings();
    JoystickButton ArmRetractButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.ArmRetract);

  }



  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
            coneFlipperUpButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.ConeFlipperUp);
            coneFlipperDownButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.ConeFlipperDown);
  }
  public static boolean GetConeFlipperUpButton() {
    return mechanismJoystick.getRawButton(Constants.GamePadConstants.ConeFlipperUp);
  }
  public static boolean GetConeFlipperDownButton() {
    return mechanismJoystick.getRawButton(Constants.GamePadConstants.ConeFlipperDown);
  }
 public static boolean GetGrabberCloseButton() {
  return mechanismJoystick.getRawButton(Constants.GamePadConstants.GrabberClose);
  
 } 
 public static boolean GetGrabberOpenButton() {
  return mechanismJoystick.getRawButton(Constants.GamePadConstants.GrabberOpen);
 } 
 public static boolean GetArmExtendButton() {
  return mechanismJoystick.getRawButton(Constants.GamePadConstants.ArmExtend);
 }
 public static boolean GetArmRetractButton() {
  return mechanismJoystick.getRawButton(Constants.GamePadConstants.ArmRetract);
 }
 public static boolean GetArmGroundButton() {
  return mechanismJoystick.getRawButton(Constants.GamePadConstants.ArmGround);
 }
 public static DutyCycleEncoder GetFlipperEncoder() {
  return coneFlipperEncoder;
 }
public static boolean GetGrabberCloseCubeButton() {
  return mechanismJoystick.getRawButton(Constants.GamePadConstants.GrabberCloseCube);
 } 

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return  autoChooser.getSelected();
  }

  

  public void disablePIDSubsystems() {
    m_GrabberWithPID.disable();
    m_ArmWithPID.disable();
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /* 
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
  */
}
