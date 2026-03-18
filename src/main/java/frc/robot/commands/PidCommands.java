package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PID.IntakeArm;

public class PidCommands extends Command {
  private final IntakeArm pid;
  private final double setpoint;

  public PidCommands(IntakeArm subsystem, double setpoint) {
    this.pid = subsystem;
    this.setpoint = setpoint;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}
  public void execute() {
    pid.setSetpoint(setpoint); 
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}