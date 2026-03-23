package frc.robot.commands;

import static frc.robot.Constants.OuttakeConstants.TARGET_FLYWHEEL_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Outtake;

public class SpinCommand extends Command {
    final Outtake outtakeSubsystem;
    final double m_RPS;

    public SpinCommand(Outtake outtake, double RPS){
        outtakeSubsystem = outtake;
        m_RPS = RPS;
    }

    public SpinCommand(Outtake outtake){
        this(outtake, TARGET_FLYWHEEL_SPEED);
    }

    @Override
    public void initialize(){
        outtakeSubsystem.setTargetVelocity(m_RPS);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        outtakeSubsystem.stopMotor();
    }
}
