// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.HandleConeFlipper;
import frc.robot.commands.HandleGrabber;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ConeFlipper;
import frc.robot.commands.AutoDriveOutOfCommunity;
import frc.robot.commands.AutoPickForTwoCubes;
import frc.robot.commands.HandleArm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Grabber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final Grabber m_Grabber = new Grabber();
  private final Arm m_Arm = new Arm();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  static Joystick mechanismJoystick = new Joystick(Constants.ButtonBoxID);
  static JoystickButton coneFlipperUpButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.ConeFlipperUp);
  static JoystickButton coneFlipperDownButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.ConeFlipperDown);
  static JoystickButton GrabberOpenButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.GrabberOpen);
  static JoystickButton GrabberCloseButton = new JoystickButton(mechanismJoystick, Constants.GamePadConstants.GrabberClose);
  private HandleConeFlipper handleConeFlipper = new HandleConeFlipper(m_ConeFlipper);
  private HandleGrabber handleGrabber = new HandleGrabber(m_Grabber);
  private HandleArm handleArm = new HandleArm(m_Arm);
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
    autoChooser.addOption("Auto Two Cubes", new AutoPickForTwoCubes(m_robotDrive));
    SmartDashboard.putData("Autonomous", autoChooser);
    //Shuffleboard.getTab("Gryo tab").add(m_robotDrive.m_gyro);

    configureButtonBindings();
    m_ConeFlipper.setDefaultCommand(handleConeFlipper);
    m_Grabber.setDefaultCommand(handleGrabber);
    m_Arm.setDefaultCommand(handleArm);
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.06),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
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
 public static DutyCycleEncoder GetFlipperEncoder() {
  return coneFlipperEncoder;
}


  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return  autoChooser.getSelected();
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
