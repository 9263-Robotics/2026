// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake i_subsystem;


  /**
   * Creates a new IntakeCommands.
   *
   * @param intake The subsystem used by this command.
   */
  public IntakeCommands(Intake intake) {
    i_subsystem = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Start the motor when the command begins

    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    i_subsystem.startIntake(); // Turns the motor on while 'B' is held
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motor when the command is finished or interrupted (button released)
    i_subsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Return false so the command keeps running as long as the button is held down
    return false;
  }
}