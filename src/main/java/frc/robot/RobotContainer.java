// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AddressableLEDs;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.ShootAtTarget;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
// import frc.robot.commands.ShootAtTarget;
// import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Turret;
// import frc.robot.subsystems.Turret;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Getter for singleton instance of RobotContainer, which can be used to access
  // subsystems in the Robot class and other places
  private static RobotContainer instance;

  public static RobotContainer getInstance() {
    return instance;
  }

  // ------- SUBSYSTEM DEFINES -------
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/comp")); // "swerve/test" or "swerve/comp" to set which swerve base we're using
  private final Kicker kicker = new Kicker();
  // private final Turret turret = new Turret(drivebase);
  private final PowerDistribution pdh = new PowerDistribution(9, ModuleType.kRev);
  private final Intake intake = new Intake();
  private final Hood hood = new Hood();
  private final Turret turret = new Turret(drivebase);
  private final Flywheel flywheel = new Flywheel();

  // Replace with CommandPS4Controller or CommandXBoxController if needed
  private final CommandPS5Controller m_driverController = new CommandPS5Controller(
      OperatorConstants.kDriverControllerPort);

  // private final CommandPS5Controller m_operatorController = new
  // CommandPS5Controller(OperatorConstants.kOperatorControllerPort);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1)
      .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(1)
      .allianceRelativeControl(true);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  public Command shakeIntake = new SequentialCommandGroup(
      intake.setSetpoint(-5).withTimeout(0.4),
      intake.setSetpoint(-15).withTimeout(0.4));

  // Define Addressable LED subsystem
  private final AddressableLEDs m_AddressableLEDs = new AddressableLEDs(true); // set to false to disable all LED code

  private final SendableChooser<Command> autoChooser;
  // List of autos to display.

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    instance = this;

    m_AddressableLEDs.setPatternMode(AddressableLEDs.PatternMode.PATTERN0); // Change to set the patternMode, disabled if AddressableLEDs.enabled is false when initiallized

    CanandEventLoop.getInstance();
    // Configure the trigger bindings
    setupNamedCommands();
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    // Intializing the list of autos to display.

    setupAutoChooser();

    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    hood.setDefaultCommand(hood.HoodDown());

    Shuffleboard.getTab("TELEM").addDouble("Match Time", () -> Timer.getMatchTime());
    Shuffleboard.getTab("TELEM").addDouble("Voltage", () -> pdh.getVoltage());
    Shuffleboard.getTab("TELEM").addDouble("Current", () -> pdh.getTotalCurrent());
    Shuffleboard.getTab("TELEM").addDouble("Power", () -> pdh.getTotalPower());

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // m_driverController.circle().whileTrue(drivebase.zeroGyro());

    // m_driverController.R2().whileTrue(kicker.shoot());
    // m_driverController.R2().whileTrue(new ShootAtTarget(turret, kicker, hood));
    // m_driverController.R1().whileTrue(flywheel.flywheelSpinup());

    // m_driverController.L1().onTrue(hood.iterateRot());
    // m_driverController.L2().whileTrue(hood.HoodDown().repeatedly());

    // m_driverController.povDown().whileTrue(intake.setSetpoint(-16));
    // m_driverController.povRight().whileTrue(intake.setSetpoint(-5));
    // m_driverController.povUp().whileTrue(intake.setSetpoint(0));

    // m_driverController.square().whileTrue(intake.runIntakeMotor());

    // m_driverController.circle().onTrue(turret.runOnce(() -> {
    // turret.setTurretAngle(turret.getTurretRotation().plus(Rotation2d.fromDegrees(5)));
    // }));

    // m_driverController.cross().onTrue(turret.runOnce(() -> {
    // turret.setTurretAngle(turret.getTurretRotation().minus(Rotation2d.fromDegrees(5)));
    // }));

    // m_driverController.cross().whileTrue( // On button press, enable LED pattern (unused)
    //     Commands.startEnd(
    //         () -> m_AddressableLEDs.setPatternMode(AddressableLEDs.PatternMode.PATTERN0),
    //         () -> m_AddressableLEDs.setPatternMode(AddressableLEDs.PatternMode.OFF),
    //         m_AddressableLEDs).ignoringDisable(true));

    // m_operatorController.touchpad().whileTrue(hood.HoodDown().repeatedly());

    m_driverController.R2()
        .whileTrue(new ShootAtTarget(turret, kicker, hood, drivebase, flywheel).alongWith(hood.HoodRunAnlge()));
    m_driverController.R1()
        .whileTrue(new AimAtTarget(turret, hood, drivebase, flywheel).alongWith(hood.HoodRunAnlge()));

    m_driverController.povDown().onTrue(intake.IntakeDown());
    m_driverController.povUp().onTrue(intake.IntakeUp());
    m_driverController.povRight().onTrue(intake.IntakeMiddle());

    // m_driverController.triangle().whileTrue(intake.runIntakeMotor());
    // m_driverController.triangle().whileTrue(kicker.shoot());

    // m_driverController.L1().whileTrue(intake.IntakeShake());
    m_driverController.L1().whileTrue(intake.IntakeShootInConstant());

    m_driverController.L2().whileTrue(intake.runIntakeWithPID());

    m_driverController.square().whileTrue(kicker.unJam());

    m_driverController.circle().whileTrue(turret.setTurretToZeroCommand());
    // m_driverController.circle().whileTrue()

    // m_driverController.cross().whileTrue(flywheel.flywheelSpinup());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }

  public void setupNamedCommands() {
    // NamedCommands.registerCommand("Testcommand", new PrintCommand("This is a test
    // command"));

    NamedCommands.registerCommand("SpinupFlywheel", flywheel.flywheelSpinup().withTimeout(0.8));

    NamedCommands.registerCommand("Shoot", kicker.shoot());

    NamedCommands.registerCommand("ShootIDK",
        flywheel.flywheelSpinup().withTimeout(0.8).andThen(kicker.shoot()).withTimeout(3));

    NamedCommands.registerCommand("IntakeDown", intake.IntakeDown().withTimeout(0.2));

    NamedCommands.registerCommand("IntakeDownRun", intake.runIntakeWithPID().withTimeout(0.2));

    NamedCommands.registerCommand("IntakeMid", intake.IntakeMiddle().withTimeout(0.2));

    NamedCommands.registerCommand("IntakeUp", intake.IntakeUp().withTimeout(0.2));

    // NamedCommands.registerCommand("ShakeIntake",
    // intake.setSetpoint(-5).withTimeout(0.4).andThen(intake.setSetpoint(-15).withTimeout(0.4)));

    NamedCommands.registerCommand("StartIntake", intake.run(intake::startIntake).withTimeout(0.1));

    NamedCommands.registerCommand("StopIntake", intake.runOnce(intake::stopIntake));

    // NamedCommands.registerCommand("ShakeIntake", Commands.none());
    NamedCommands.registerCommand("ShakeIntake", intake.IntakeShake());

    NamedCommands.registerCommand("ShootAtTarget", new ShootAtTarget(turret, kicker, hood, drivebase, flywheel));
    NamedCommands.registerCommand("AimAtTarget", new AimAtTarget(turret, hood, drivebase, flywheel));

    // First argument is the name of the command PathPlanner will use. Second
    // argument is the actual command WITH parameters the robot will run.
  }

  public AddressableLEDs getAddressableLEDs() {
    return m_AddressableLEDs;
  }

  private void setupAutoChooser() {
    // new PathPlannerAuto("Testauto"); //idk if this is actually nessessary lol, I
    // think it worked without it last year, but we had it
    autoChooser.addOption("Just Shoot v2", flywheel.flywheelSpinup().withTimeout(0.8).andThen(kicker.shoot()));
    Shuffleboard.getTab("AUTO").add("Auto Select", autoChooser);
    // Displays the dropdown menu for selecting the auto. (Use elastic?)
  }
}
