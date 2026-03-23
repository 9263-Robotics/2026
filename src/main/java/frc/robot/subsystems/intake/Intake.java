package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.CANIDs;

public class Intake extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(CANIDs.IntakeMotor);

  /** Creates a new ExampleSubsystem. */
  public Intake() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {

        });
  }

public void startIntake(){
    intakeMotor.set(1.0); //full speed
}
public void stopIntake(){
    intakeMotor.set(0.0); //stop
}
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
