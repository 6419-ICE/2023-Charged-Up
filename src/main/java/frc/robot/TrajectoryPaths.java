package frc.robot;
import java.util.List;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class TrajectoryPaths {
    
    //private DriveSubsystem m_driveSubSystem; 

     // Create config for trajectory
  static TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);


    public TrajectoryPaths()
    {}
    // Start Putting new Autonomus Trajectories aqui(here), use this entry as an example. 
    public static Trajectory trajectoryExample () {

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(3, 0, new Rotation2d(0)),
    config);

    return trajectory; 
    }

    public static Trajectory trajectoryAutoDriveOutOfCommunity () {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0,0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1, 0, new Rotation2d(0)),
        config);

        return trajectory; 
    }

    // These sets of trajectories are being used to Create the Two Cube Autonomus Program
    public static Trajectory trajectoryAutoForwardTowardsSecondBlock () {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1,0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(241), 0, Rotation2d.fromDegrees(179.4)),
            config);
    
            return trajectory; 
        }
    

    public static Trajectory trajectoryAutoBackTowardsDropOffOfSecondBlock () {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1,0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(224), 0, Rotation2d.fromDegrees(179.4)),
            config);
    
            return trajectory; 
        }
    

    // Trajectory to get onto the charging station
    public static Trajectory trajectoryAutoEngageOnChargingStation () {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0.25,0)),
            // End 100 "meters" straight ahead of where we started, facing forward
            new Pose2d(0.5, 0, new Rotation2d(0)),
            config);
    
            return trajectory; 
    }

    public static Trajectory trajectoryAutoDriveOutLeft() {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d((Units.feetToMeters(22)),0), 
            new Translation2d((Units.feetToMeters(22)),Units.feetToMeters(3.75)), 
            new Translation2d((Units.inchesToMeters(40)),Units.feetToMeters(3.75))
            ),
            // End 14 straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(43), Units.feetToMeters(6.00), new Rotation2d(Units.degreesToRadians(0))),
            config);
        return trajectory; 
    }

    public static Trajectory trajectoryAutoDriveOutRight() {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d((Units.feetToMeters(22)),0), 
            new Translation2d((Units.feetToMeters(22)),Units.feetToMeters(-6.0)), 
            new Translation2d((Units.inchesToMeters(38)),Units.feetToMeters(-6.0))
            ),
            // End 14 straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(41), Units.feetToMeters(-6.0), new Rotation2d(Units.degreesToRadians(0))),
            config);
        return trajectory; 
    }
    
    public static Trajectory trajectoryAutoDriveOutCenter() {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d((Units.feetToMeters(22)),0)),
            // End 14 straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(43), Units.feetToMeters(6.00), new Rotation2d(Units.degreesToRadians(0))),
            config);
            return trajectory; 
    }


    public static Trajectory trajectoryMoveToCube() {
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0,0, new Rotation2d(0)), 
      List.of(
    //new Translation2d(Units.inchesToMeters(100), 0),
    new Translation2d(Units.inchesToMeters(200), 0)
    //new Translation2d(Units.inchesToMeters(300), 0)
    ),
      new Pose2d(Units.inchesToMeters(224),0,new Rotation2d(0)), 
      config
      );
      
      
        return trajectory;
    }
    
}
