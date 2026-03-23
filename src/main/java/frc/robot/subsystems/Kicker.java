package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.KickerConstants.*;

public class Kicker extends SubsystemBase {
    public final TalonFX motor = new TalonFX(MOTORCANID);
}
