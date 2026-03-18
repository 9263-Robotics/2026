package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Setpoints;
import frc.robot.commands.Turret.SimpleAimAtTarget;
import frc.robot.subsystems.PID.IntakeArm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.util.StaticPoses;

public class StateManager extends SubsystemBase {
    //TODO: Create button bindings for the four modes, as well as switching the target trench
    public enum State {
        TRENCH,
        INTAKE,
        SHOOT,
        PASS,
    }

    final Outtake outtakeSubsystem;
    final Turret turretSubsytem;
    final SwerveSubsystem swerveSubsystem;
    final Intake intakeSubsystem;
    final IntakeArm intakeArmSubsytem;
    final Vision visionSubsystem;

    public boolean passingScoringTableSide;
    
    State state;

    public StateManager(Outtake outtake, Turret turret, SwerveSubsystem swerve, Intake intake, IntakeArm intakeArm, Vision vision){
        outtakeSubsystem = outtake;
        turretSubsytem = turret;
        swerveSubsystem = swerve;
        intakeSubsystem = intake;
        intakeArmSubsytem = intakeArm;
        visionSubsystem = vision;
        BindStateActions();
    }

    void BindStateActions(){
        new Trigger(() -> (state == State.TRENCH)).whileTrue(TrenchStateCommand());
        new Trigger(() -> (state == State.INTAKE)).whileTrue(IntakeStateCommand());
        new Trigger(() -> (state == State.SHOOT)).whileTrue(ShootStateCommand());
        new Trigger(() -> (state == State.PASS)).whileTrue(PassStateCommand());
    }

    Command TrenchStateCommand() {
        return runOnce(() -> {
            // TODO: set hood to stowed position
        });
    }

    Command IntakeStateCommand() {
        return runEnd(() -> {
            intakeArmSubsytem.setSetpoint(Setpoints.IntakeArm.ENGAGED);
            intakeSubsystem.startIntake();
        }, () -> {
            intakeArmSubsytem.setSetpoint(Setpoints.IntakeArm.STOWED);
            intakeSubsystem.stopIntake();
        });
    }

    Command ShootStateCommand() {
        return defer(() -> {
            // TODO: get flywheels up to speed, turn on kicker and spindexer if at speed
            Optional<Pose2d> hubPose = StaticPoses.GetHubPoseOptional();
            if (hubPose.isPresent()){
                return new SimpleAimAtTarget(swerveSubsystem, visionSubsystem, turretSubsytem, hubPose.get());
            } else {
                return new PrintCommand("switched to shooting state but unable to locate hub");
            }
        });
    }

    Command PassStateCommand() {
        return defer(() -> {
            // TODO: get flywheels up to speed, turn on kicker and spindexer if at speed
            Optional<Pose2d> targetPose = passingScoringTableSide ? StaticPoses.GetScoringTableTrenchOptional() : StaticPoses.GetNonScoringTableTrenchOptional();
            if (targetPose.isPresent()){
                return new SimpleAimAtTarget(swerveSubsystem, visionSubsystem, turretSubsytem, targetPose.get());
            } else {
                return new PrintCommand("switched to pass state but unable to locate trench");
            }
        });
    }
}
