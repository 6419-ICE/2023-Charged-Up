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

    // Trajectory to get onto the charging station
    public static Trajectory trajectoryAutoEngageOnChargingStation () {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0,0)),
            // End 100 inches straight ahead of where we started, facing forward
            new Pose2d((Units.inchesToMeters(100)), 0, new Rotation2d(0)),
            config);
    
            return trajectory; 
        }
}
