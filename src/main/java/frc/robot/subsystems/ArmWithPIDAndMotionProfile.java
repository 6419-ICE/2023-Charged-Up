package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstantsForPIDAndMotionProfile;

public class ArmWithPIDAndMotionProfile extends ProfiledPIDSubsystem {
    private TalonFX ArmMotor = new TalonFX(Constants.ArmConstantsForPIDAndMotionProfile.kMotorPort);
    private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
        ArmConstantsForPIDAndMotionProfile.kSVolts, ArmConstantsForPIDAndMotionProfile.kGVolts,
        ArmConstantsForPIDAndMotionProfile.kVVoltSecondPerRad, ArmConstantsForPIDAndMotionProfile.kAVoltSecondSquaredPerRad);

    enum statesForArm {
      IDLE,   
      Home,
        Top,
        Ground
        }
    statesForArm currentStateForArm = statesForArm.IDLE;         

  /** Create a new ArmSubsystem. */
  public ArmWithPIDAndMotionProfile() {
    super(
        new ProfiledPIDController(
          ArmConstantsForPIDAndMotionProfile.kP,
            0,
            ArmConstantsForPIDAndMotionProfile.kd,
            new TrapezoidProfile.Constraints(
              ArmConstantsForPIDAndMotionProfile.kMaxVelocityRadPerSecond,
                ArmConstantsForPIDAndMotionProfile.kMaxAccelerationRadPerSecSquared)),
        0);
   // m_encoder.setDistancePerPulse(ArmConstantsForPIDAndMotionProfile.kEncoderDistancePerPulse);
    // Start arm at rest in neutral position
    //setGoal(ArmConstantsForPIDAndMotionProfile.kArmOffsetRads);
  }
  public double goalInAuto= Math.toRadians(280);

  public void setGoal(double goal) {
    if (goal > Constants.ArmConstantsForPIDAndMotionProfile.kArmMaxOffsetRads) {
        goal = Constants.ArmConstantsForPIDAndMotionProfile.kArmMaxOffsetRads;
    } else if (goal < Constants.ArmConstantsForPIDAndMotionProfile.kArmMinOffsetRads) {
        goal = Constants.ArmConstantsForPIDAndMotionProfile.kArmMinOffsetRads;
    }
    goalInAuto = goal; 
    super.setGoal(goal);
    System.out.println("Goal We are Going To" + goal); 
    //this.disable();
  }

  public double getGoalForAuto()
  {
    return goalInAuto;
  }
  public void Home(){
    currentStateForArm = statesForArm.Home;
    setGoal(Math.toRadians(Constants.ArmConstantsForPIDAndMotionProfile.homePosition));
    this.enable(); 
}
public void Top(){
    currentStateForArm = statesForArm.Top;
    setGoal(Math.toRadians(Constants.ArmConstantsForPIDAndMotionProfile.dropPosition));
    this.enable(); 
}
public void Ground(){
  currentStateForArm = statesForArm.Ground;
  setGoal(Math.toRadians(Constants.ArmConstantsForPIDAndMotionProfile.groundPosition));
  this.enable(); 
}

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    ArmMotor.set(TalonFXControlMode.PercentOutput, -output);//+ feedforward ); 
    System.out.println("Current Ouput To Arm Motor: " + -output + " Curent Angle: " + setpoint.position);
    //m_motor.setVoltage(output + feedforward);
  }
//private double testRadians = 0.0;

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("PositionInDegreesArm", Math.toDegrees(this.getMeasurement()));
    SmartDashboard.putNumber("GoalPosition", Math.toDegrees(this.getGoalForAuto()));

    super.periodic();

  }

  @Override
  public void getEncoderPos() {
    return ArmMotor.getSensorCollection().getIntegratedSensorPosition();
  }
  public double getMeasurement() {
    return -Math.toRadians(ArmMotor.getSensorCollection().getIntegratedSensorPosition() / Constants.ArmConstantsForPIDAndMotionProfile.ArmUnitsPerDegree); //+ ArmConstantsForPIDAndMotionProfile.kArmOffsetRads;
  /* 
    if((-testRadians) > this.m_controller.getGoal().position)
        testRadians+=0.10;
      else
        testRadians-=0.10;
    return -testRadians;
    */
  }
}