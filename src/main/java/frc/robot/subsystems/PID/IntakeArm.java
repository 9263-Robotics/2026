package frc.robot.subsystems.PID;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class IntakeArm extends SubsystemBase {
  
  //TODO IDK THE ACTUAL CAN ID STUFF SO CHANGE IT
  private final SparkMax pidMotor1 = new SparkMax(14, MotorType.kBrushless);
  private final SparkMax pidMotor2 = new SparkMax(15, MotorType.kBrushless);  

  private final RelativeEncoder encoder = pidMotor1.getEncoder();

  private final PIDController pid = new PIDController(0.00025, 0.00001, 0.00001);
  private double setpoint = 0.0;

  public IntakeArm() {
    encoder.setPosition(0);
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