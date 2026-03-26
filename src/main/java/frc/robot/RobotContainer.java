// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.commands.SpinCommand;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // ------- SUBSYSTEM DEFINES -------
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/comp") // "swerve/test" or "swerve/comp" to set which swerve base we're using

  ); 
  // private final Intake m_intake = new Intake();
  // private final IntakeArm pid = new IntakeArm();
  private final Outtake outtake = new Outtake();
  private final Turret turret = new Turret(drivebase);
  private final Kicker kicker = new Kicker();
  private final Spindexer spindexer = new Spindexer();
  private final Hood hood = new Hood();

  // Replace with CommandPS4Controller or CommandXBoxController if needed
  private final CommandPS5Controller m_driverController = new CommandPS5Controller(OperatorConstants.kDriverControllerPort);


   /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> m_driverController.getRightX() *-1 )
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1)
                                                            .allianceRelativeControl(true);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);                                                          

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register commands for PathPlanner

    // Movement stuff
    // NamedCommands.registerCommand("Trench", stateManager.SetState(State.TRENCH));
    // NamedCommands.registerCommand("passScoringSide", stateManager.SetState(State.PASS_SCORING_SIDE));
    // NamedCommands.registerCommand("passScoringSideIntake", stateManager.SetState(State.PASS_SCORING_SIDE_INTAKE));
    // NamedCommands.registerCommand("passNonscoringSide", stateManager.SetState(State.PASS_NONSCORING_SIDE));
    // NamedCommands.registerCommand("passNonscoringSideIntake", stateManager.SetState(State.PASS_NONSCORING_SIDE_INTAKE));

    // Configure the trigger bindings
    configureBindings();

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.circle().onTrue(drivebase.zeroGyro());

    m_driverController.square().whileTrue(new ParallelCommandGroup(
      new SpinCommand(outtake),
      outtake.runEnd(()->{
        if (outtake.atSetpoint()){
          spindexer.motor.set(SpindexerConstants.MOTORSPEED);
          kicker.motor.set(KickerConstants.MOTORSPEED);
        }
      },()->{
        spindexer.motor.stopMotor();
        kicker.motor.stopMotor();
      })
      
    ));

    m_driverController.L1().whileTrue(new RunCommand(() -> {
      turret.setTurretAngle(turret.getTurretRotation().plus(Rotation2d.fromDegrees(2)));
    }));

    m_driverController.R1().whileTrue(new RunCommand(() -> {
      turret.setTurretAngle(turret.getTurretRotation().minus(Rotation2d.fromDegrees(2)));
    }));

    // move hood up on left button
    m_driverController.L2().whileTrue(new RunCommand(() -> {
      hood.setHoodAngle(() -> hood.getSetpoint()+0.01);
    }));

    //move hood down on right button
    m_driverController.R2().onTrue(new RunCommand(() -> {
      hood.setHoodAngle(() -> hood.getSetpoint()-0.01);
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return Commands.none();
  }
}
