package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.KickerConstants;
import frc.robot.Constants.Setpoints;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.commands.SpinCommand;
import frc.robot.commands.Turret.ShootAtTarget;
import frc.robot.commands.Turret.SimpleAimAtTarget;
import frc.robot.subsystems.PID.IntakeArm;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.util.StaticPoses;

public class StateManager extends SubsystemBase {
    public enum State {
        TRENCH,
        INTAKE,
        SHOOT,
        SHOOT_AND_INTAKE,
        PASS_SCORING_SIDE,
        PASS_SCORING_SIDE_INTAKE,
        PASS_NONSCORING_SIDE,
        PASS_NONSCORING_SIDE_INTAKE,
    }

    public boolean IsShooting(){
        return (
               state == State.SHOOT
            || state == State.SHOOT_AND_INTAKE
            || state == State.PASS_SCORING_SIDE
            || state == State.PASS_SCORING_SIDE_INTAKE
            || state == State.PASS_NONSCORING_SIDE
            || state == State.PASS_NONSCORING_SIDE_INTAKE
        );
    }

    public Command SetState(State newState){
        return runOnce(() -> {
            state = newState;
        });
    }

    final Outtake outtakeSubsystem;
    final Turret turretSubsytem;
    final SwerveSubsystem swerveSubsystem;
    final Intake intakeSubsystem;
    final IntakeArm intakeArmSubsytem;
    final Vision visionSubsystem;
    final Kicker kickerSubsystem;
    final Spindexer spindexerSubsystem;
    final Hood hoodSubsystem;
    
    public State state;

    public StateManager(
            Outtake outtake, 
            Turret turret, 
            SwerveSubsystem swerve, 
            Intake intake, 
            IntakeArm intakeArm, 
            Vision vision,
            Kicker kicker,
            Spindexer spindexer,
            Hood hood
            ){

        outtakeSubsystem = outtake;
        turretSubsytem = turret;
        swerveSubsystem = swerve;
        intakeSubsystem = intake;
        intakeArmSubsytem = intakeArm;
        visionSubsystem = vision;
        spindexerSubsystem = spindexer;
        kickerSubsystem = kicker;
        hoodSubsystem = hood;
        BindStateActions();
    }

    void BindStateActions(){
        new Trigger(() -> (state == State.TRENCH)).whileTrue(TrenchStateCommand());
        new Trigger(() -> (state == State.INTAKE)).whileTrue(IntakeStateCommand());
        new Trigger(() -> (state == State.SHOOT)).whileTrue(ShootStateCommand());
        new Trigger(() -> (state == State.SHOOT_AND_INTAKE)).whileTrue(new ParallelCommandGroup(ShootStateCommand(), IntakeStateCommand()));
        new Trigger(() -> (state == State.PASS_SCORING_SIDE)).whileTrue(PassStateCommand(true));
        new Trigger(() -> (state == State.PASS_SCORING_SIDE_INTAKE)).whileTrue(new ParallelCommandGroup(PassStateCommand(true), IntakeStateCommand()));
        new Trigger(() -> (state == State.PASS_NONSCORING_SIDE)).whileTrue(PassStateCommand(false));
        new Trigger(() -> (state == State.PASS_NONSCORING_SIDE_INTAKE)).whileTrue(new ParallelCommandGroup(PassStateCommand(false), IntakeStateCommand()));
    }

    Command TrenchStateCommand() {
        return hoodSubsystem.setHoodAngle(Setpoints.Hood.STOWED);
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
        final Command aimCommand = new ShootAtTarget(turretSubsytem, swerveSubsystem, outtakeSubsystem, hoodSubsystem);
        // Optional<Pose2d> hubPose = StaticPoses.GetHubPoseOptional();
        // if (hubPose.isPresent()){
        //     aimCommand = new ShootAtTarget(turretSubsytem, swerveSubsystem);
        // } else {
        //     aimCommand = new PrintCommand("switched to shooting state but unable to locate hub");
        // }

        return new ParallelCommandGroup(
            aimCommand, 
            ConditionalStartShooting(),
            new SpinCommand(outtakeSubsystem)
        );
    }

    Command ConditionalStartShooting() {
        return runEnd(() -> {
            if (outtakeSubsystem.atSetpoint() && turretSubsytem.isTurretAligned()){
                kickerSubsystem.motor.set(KickerConstants.MOTORSPEED);
                spindexerSubsystem.motor.set(SpindexerConstants.MOTORSPEED);
            }
        }, () -> {
            kickerSubsystem.motor.set(0);
            spindexerSubsystem.motor.set(0);
        });
    }

    Command PassStateCommand(boolean passingScoringTableSide) {
        final Command aimCommand;
        Optional<Pose2d> targetPose = passingScoringTableSide ? StaticPoses.GetScoringTableTrenchOptional() : StaticPoses.GetNonScoringTableTrenchOptional();
        if (targetPose.isPresent()){
            aimCommand = new SimpleAimAtTarget(swerveSubsystem, visionSubsystem, turretSubsytem, targetPose.get());
        } else {
            aimCommand = new PrintCommand("switched to pass state but unable to locate trench");
        }

        return new ParallelCommandGroup(
            new SpinCommand(outtakeSubsystem),
            ConditionalStartShooting(),
            aimCommand
        );
    }
}
