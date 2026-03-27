package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class Intake extends SubsystemBase {
  private final TalonFX intakeMotor = new TalonFX(14);

  private final SparkMax pidMotor1 = new SparkMax(12, MotorType.kBrushless);
  private final SparkMax pidMotor2 = new SparkMax(13, MotorType.kBrushless);  

  private final RelativeEncoder encoder = pidMotor1.getEncoder();

  private final PIDController pid = new PIDController(0.00025, 0.00001, 0.00001);
  private double setpoint = 0.0;

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    encoder.setPosition(0);
  }

  public void startIntake(){
      intakeMotor.set(1.0); //full speed
  }

  public void stopIntake(){
      intakeMotor.set(0.0); //stop
  }

  public double getPosition() {
    return encoder.getPosition() * 360;  
  }

  @Override
  public void periodic() {
    double output = pid.calculate(getPosition(), setpoint); // takes the current position and the desired setpoint and calculates the output
    output = MathUtil.clamp(output, -0.5, 0.5); // limits the output to be between -0.5 and 0.5
    pidMotor1.set(output); // sets the motor output to the calculated value
    pidMotor2.set(-output); //negative cuz facing other way

  }
  public void setSetpoint(double point){
    setpoint = point; //this is the method that will be called to change the setpoint of the PID controller
  }
}
