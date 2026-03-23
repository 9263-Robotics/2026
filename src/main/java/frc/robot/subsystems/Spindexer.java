package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SpindexerConstants.*;

public class Spindexer extends SubsystemBase {
    public final SparkMax motor = new SparkMax(MOTORCANID, MotorType.kBrushless);
}
