package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {
  public final TalonFX kicker = new TalonFX(15);

  // private final TalonFX FlywheelTop = new TalonFX(18);
  // private final TalonFX FlywheelBottom = new TalonFX(19);

  public final SparkFlex Spindexer = new SparkFlex(11, MotorType.kBrushless);

  private final TalonFX intake = new TalonFX(14);

  // private final Slot0Configs configs = new Slot0Configs();
  // final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

  // private final PIDController pid = new PIDController(0, 0.0, 0.0);

  private double DesiredFlywheelSpeed;



  /** Creates a new ExampleSubsystem. */
  public Kicker() {
    
    // FlywheelBottom.setControl(new Follower(18, MotorAlignmentValue.Aligned));
    // configs.kP = 0.3;
    // configs.kI = 0;
    // configs.kD = 0;
    // configs.kV = 0.13;
    // FlywheelTop.getConfigurator().apply(configs);
    // DesiredFlywheelSpeed = 10;
    // Shuffleboard.getTab(getName()).add("Flyhweel Setpoint", pid);
    // Shuffleboard.getTab(getName()).addDouble("Flywheel Setpoint", () -> DesiredFlywheelSpeed);
    // Shuffleboard.getTab(getName()).addDouble("FlywheelOutput", () -> FlywheelTop.get());
    // Shuffleboard.getTab(getName()).addDouble("Flywheel Speed", () -> FlywheelTop.getVelocity().getValueAsDouble() * (4/3) *60);
    // SmartDashboard.putNumber("Flyhweel AHAHJJHA", DesiredFlywheelSpeed);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Flywheel Setpoint");
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */


public void startIntake(){
    kicker.set(1.0); //full speed
}
public void stopIntake(){
    kicker.set(0.0); //stop
}

public Command shoot() {
    return runEnd(() -> {
        Spindexer.set(-0.5);
        kicker.set(0.4);
    }, () -> {
        Spindexer.set(0);
        kicker.set(0);
    });
}

public Command unJam() {
  return runEnd(() -> {
    kicker.set(-0.2);
    Spindexer.set(0.3);
  }, () -> {
    kicker.set(0);
  });
}

}