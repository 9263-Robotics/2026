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

public class Flywheel extends SubsystemBase {
  public final TalonFX kicker = new TalonFX(15);

  private final TalonFX FlywheelBottom = new TalonFX(19);
  private final TalonFX FlywheelTop = new TalonFX(18);


  private final Slot0Configs configs = new Slot0Configs();
  final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

  private final PIDController pid = new PIDController(0, 0.0, 0.0);

  private double DesiredFlywheelSpeed;



  /** Creates a new ExampleSubsystem. */
  public Flywheel() {
    
    FlywheelTop.setControl(new Follower(19, MotorAlignmentValue.Aligned));
    configs.kP = 0.3;
    configs.kI = 0;
    configs.kD = 0;
    configs.kV = 0.13;

    
    FlywheelBottom.getConfigurator().apply(configs);
    DesiredFlywheelSpeed = -2500;


    // Shuffleboard.getTab(getName()).add("Flyhweel Setpoint", pid);
    // Shuffleboard.getTab(getName()).addDouble("Flywheel Setpoint", () -> DesiredFlywheelSpeed);
    // Shuffleboard.getTab(getName()).addDouble("FlywheelOutput", () -> FlywheelTop.get());
    Shuffleboard.getTab(getName()).addDouble("Flywheel Speed", () -> FlywheelBottom.getVelocity().getValueAsDouble() * (4/3) *60);
    // Shuffleboard.getTab(getName()).addDouble("Flywheel Speed", () -> FlywheelTop.getVelocity().getValueAsDouble() * (4/3) *60);
    SmartDashboard.putNumber("Flyhweel AHAHJJHA", DesiredFlywheelSpeed);
    // Shuffleboard.getTab(getName()).addDouble("Flywheel Setpoint", () -> FlywheelBottom.getControlMode().getValueAsDouble());

    Shuffleboard.getTab(getName()).addBoolean("Flywheel At Setpoint", this::atSetpoint);
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



  public Command flywheelSpinup() {
    return runEnd(() -> {
        setTargetVelocity(SmartDashboard.getNumber("Flyhweel AHAHJJHA", DesiredFlywheelSpeed));
    }, () -> {
        FlywheelBottom.set(-0);
        setTargetVelocity(0);
    });
  }

  public Command setFlywheelSpeed(double RPM) {
    return run( () -> {
      setTargetVelocity(RPM);
    });
  }

  public void setTargetVelocity(double RPM){
      FlywheelBottom.setControl(request.withVelocity(RPM/60 / (4/3)));
  }

  public Command manualFlywheelCommand() {
    return runEnd(() -> {
      setTargetVelocity(-1900);
    }, 
    () -> {
      setTargetVelocity(0);
    });
  }

  public boolean atSetpoint() {

      return (10 > Math.abs(FlywheelBottom.getClosedLoopError().getValueAsDouble())); //1.0 as tolerance
  }
}