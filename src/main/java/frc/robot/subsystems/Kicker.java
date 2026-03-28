package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Kicker extends SubsystemBase {
  private final TalonFX kicker = new TalonFX(15);

  private final TalonFX FlywheelTop = new TalonFX(18);
  private final TalonFX FlywheelBottom = new TalonFX(19);

  private final SparkFlex Spindexer = new SparkFlex(11, MotorType.kBrushless);

  private final TalonFX intake = new TalonFX(14);

  /** Creates a new ExampleSubsystem. */
  public Kicker() {
    FlywheelBottom.setControl(new Follower(18, MotorAlignmentValue.Aligned));

    
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
        FlywheelTop.set(-0.9);
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


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}