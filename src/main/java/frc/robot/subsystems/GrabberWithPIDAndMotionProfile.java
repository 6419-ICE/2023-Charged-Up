package frc.robot.subsystems;
import javax.crypto.interfaces.PBEKey;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.GrabberConstantsForPIDAndMotionProfile;

public class GrabberWithPIDAndMotionProfile extends ProfiledPIDSubsystem {
    private TalonFX GrabberMotor = new TalonFX(Constants.GrabberConstantsForPIDAndMotionProfile.kMotorPort);
    private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
        GrabberConstantsForPIDAndMotionProfile.kSVolts, GrabberConstantsForPIDAndMotionProfile.kGVolts,
        GrabberConstantsForPIDAndMotionProfile.kVVoltSecondPerRad, GrabberConstantsForPIDAndMotionProfile.kAVoltSecondSquaredPerRad);

    enum statesForGrabber {
        Idle, 
        Open,
        CloseOnCone,
        CloseOnCube,
        CloseFully
        }
    statesForGrabber currentStateForGrabber = statesForGrabber.Idle;         

  /** Create a new ArmSubsystem. */
  public GrabberWithPIDAndMotionProfile() {
    super(
        new ProfiledPIDController(
            GrabberConstantsForPIDAndMotionProfile.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                GrabberConstantsForPIDAndMotionProfile.kMaxVelocityRadPerSecond,
                GrabberConstantsForPIDAndMotionProfile.kMaxAccelerationRadPerSecSquared)),
        0);
   // m_encoder.setDistancePerPulse(GrabberConstantsForPIDAndMotionProfile.kEncoderDistancePerPulse);
    // Start arm at rest in neutral position
    //setGoal(GrabberConstantsForPIDAndMotionProfile.kArmOffsetRads);
  }

  public void setGoal(double goal) {
    if (goal > Constants.GrabberConstantsForPIDAndMotionProfile.kArmMaxOffsetRads) {
        goal = Constants.GrabberConstantsForPIDAndMotionProfile.kArmMaxOffsetRads;
    } else if (goal < Constants.GrabberConstantsForPIDAndMotionProfile.kArmMinOffsetRads) {
        goal = Constants.GrabberConstantsForPIDAndMotionProfile.kArmMinOffsetRads;
    }
    super.setGoal(goal);
    //this.disable();
  }
  public void openGripper(){
    currentStateForGrabber = statesForGrabber.Open;
    setGoal(Math.toRadians(90));
    this.enable(); 
}
public void CloseOnCone(){
    currentStateForGrabber = statesForGrabber.Open;
    setGoal(Math.toRadians(10));
    this.enable(); 
}
public void CloseOnCube(){
    currentStateForGrabber = statesForGrabber.Open;
    setGoal(Math.toRadians(20));
    this.enable(); 
}
public void CloseFully(){
    currentStateForGrabber = statesForGrabber.Open;
    setGoal(Math.toRadians(0));
    this.enable();
} 






  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    GrabberMotor.set(TalonFXControlMode.PercentOutput, output);//+ feedforward ); 
    //m_motor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return Math.toRadians(GrabberMotor.getSensorCollection().getIntegratedSensorPosition() / Constants.GrabberConstantsForPIDAndMotionProfile.GrabberUnitsPerDegree); //+ GrabberConstantsForPIDAndMotionProfile.kArmOffsetRads;
  }
}