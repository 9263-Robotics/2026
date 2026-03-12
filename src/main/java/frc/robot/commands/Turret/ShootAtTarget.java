// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import java.util.Arrays;
import java.util.Map;

import org.opencv.core.Point;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import swervelib.SwerveDrive;
import frc.util.PolynomialRegression;

/** An example command that uses an example subsystem. */
public class ShootAtTarget extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")

  public static PolynomialRegression shooterSpeedInterpolation = 
    new PolynomialRegression(
      Arrays.asList(
        new Point(2, 47),
        new Point(3, 50),
        new Point(4, 50),
        new Point(5, 55),
        new Point(6, 62)
      ),
       1);

  private final SwerveDrive drivebase;
  private final Turret turret;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
   public ShootAtTarget(Turret turret, SwerveDrive drivebase) {
    this.turret = turret;
    this.drivebase = drivebase;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_turret.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}