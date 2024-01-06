package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends CommandBase{
    public enum state {
        idle,
        cubeIn,
        cubeOut,
        coneIn,
        coneOut
    }
    state currentState = state.idle;
    // cube in/cone out
    IntakeSubsystem intakeSub;
    boolean isFinished = false;
    public Intake(IntakeSubsystem intakeSub) {
        this.intakeSub = intakeSub;
        addRequirements(intakeSub);
    }
    @Override
    public void initialize() {
        //intakeSub.setSpeed(0.25);
    }
    @Override 
    public void execute() {
        if (RobotContainer.GetGrabberCloseButton()) setState(state.coneIn);
        else if (RobotContainer.GetGrabberOpenButton()) setState(state.cubeIn);
        else setState(state.idle);
        if (currentState == state.idle) {
            intakeSub.setSpeed(0);
        }
        else if (currentState == state.coneIn || currentState == state.cubeOut) {
            intakeSub.setSpeed(-0.4);
        } else {
            intakeSub.setSpeed(0.4);
        }
    }
    @Override
    public void end(boolean interrupted) {
        intakeSub.setSpeed(0);
        super.end(interrupted);
    }
    @Override
    public boolean isFinished() {
        return isFinished;
    }
    public void setState(state s) {
        currentState = s;
    }
}
