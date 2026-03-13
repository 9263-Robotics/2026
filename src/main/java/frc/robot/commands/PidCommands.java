package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PidSubsystem;

public class PidCommands extends Command {
  private final PidSubsystem pid;
  private final double setpoint;

  public PidCommands(PidSubsystem subsystem, double setpoint) {
    this.pid = subsystem;
    this.setpoint = setpoint;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // pid.setSetpoint(setpoint); 
  }
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