package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Kicker extends SubsystemBase {
  public final TalonFX kicker = new TalonFX(15);

  private final TalonFX FlywheelTop = new TalonFX(18);
  private final TalonFX FlywheelBottom = new TalonFX(19);

  public final SparkFlex Spindexer = new SparkFlex(11, MotorType.kBrushless);

  private final TalonFX intake = new TalonFX(14);

  private final Slot0Configs configs = new Slot0Configs();
  final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

  private final PIDController pid = new PIDController(0.0, 0.0, 0.0);



  /** Creates a new ExampleSubsystem. */
  public Kicker() {
    FlywheelTop.getConfigurator().apply(configs);
    FlywheelBottom.setControl(new Follower(18, MotorAlignmentValue.Aligned));
    configs.kP = pid.getP();
    configs.kI = pid.getI();
    configs.kD = pid.getD();
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

public Command shoot(double flyWheelSpeed) {
    return runEnd(() -> {
        Spindexer.set(-0.5);
        kicker.set(0.4);
        setTargetVelocity(flyWheelSpeed);
    }, () -> {
        Spindexer.set(0);
        kicker.set(0);
        FlywheelTop.set(-0);
    });
}

public Command unJam() {
  return runEnd(() -> {
    kicker.set(-0.2);
  }, () -> {
    kicker.set(0);
  });
}

  public Command flywheel() {
    return runEnd(() -> {
        FlywheelTop.set(-0.9);
    }, () -> {
        FlywheelTop.set(-0);
    });
  }

  public void setTargetVelocity(double RPS){
      FlywheelTop.setControl(request.withVelocity(RPS / 1.2));
  }

  public boolean atSetpoint() {
      return 1.0 > Math.abs(FlywheelTop.getClosedLoopError().getValueAsDouble()); //1.0 as tolerance
  }
}