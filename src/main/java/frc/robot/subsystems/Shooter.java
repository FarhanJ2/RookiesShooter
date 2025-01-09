// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final PIDController topFlywheelPID = 
    new PIDController(0.01, 0, 0);

  private final PIDController bottomFlywheelPID =
    new PIDController(0.01, 0, 0);

  private final ProfiledPIDController controller =
    new ProfiledPIDController(5, 0, 0, 
      new Constraints(6, 8));

  private final SimpleMotorFeedforward bottomFlywheelFeedForward =
    new SimpleMotorFeedforward(0.13122,0.00198405);
  
  private final SimpleMotorFeedforward topFlywheelFeedForward =
    new SimpleMotorFeedforward(0.19655, 0.00212586);
  

  private final ArmFeedforward feedforward = 
    new ArmFeedforward(0.09256, 0.15116, 1.593);

  private final CANcoder encoder = new CANcoder(19, "canivore");  
  private final TalonFX pivotMotor = new TalonFX(18, "canivore");
  private final TalonFX feederMotor = new TalonFX(17,"canivore");
  private final TalonFX bottomShooterMotor = new TalonFX(15, "canivore");
  private final TalonFX topShooterMotor = new TalonFX(16, "canivore");
  /** Creates a new Shooter. */
  public Shooter() {
    feederMotor.setInverted(true);
    bottomShooterMotor.setInverted(false);
    topShooterMotor.setInverted(false);
    topFlywheelPID.setTolerance(50);
    bottomFlywheelPID.setTolerance(50);

    controller.setGoal(1.03);
  }

  public double getTopFlywheelRPM(){
    return topShooterMotor.getVelocity().getValue() * 60;
  }

  public double getBottomFlywheelRPM(){
    return bottomShooterMotor.getVelocity().getValue() * 60;
  }
  

  public void setFeederSpeed(double feederSpeed) {
    feederMotor.set(feederSpeed);
  }

  public void stopMotor() {
    bottomShooterMotor.stopMotor();
    topShooterMotor.stopMotor();
    feederMotor.stopMotor();
  }

  public void runFlywheels (double topRPM, double bottomRPM) {
    double topFeedForward = topFlywheelFeedForward.calculate(topRPM);
    double topPidOutput = topFlywheelPID.calculate(getTopFlywheelRPM(), topRPM);

    double bottomFeedForward = bottomFlywheelFeedForward.calculate(bottomRPM);
    double bottomPidOutput = bottomFlywheelPID.calculate(getBottomFlywheelRPM(), bottomRPM);

    topShooterMotor.setVoltage(topFeedForward + topPidOutput);
    bottomShooterMotor.setVoltage(bottomFeedForward + bottomPidOutput);
  }

  private double getPosition() {
    double degrees = encoder.getAbsolutePosition().getValueAsDouble() * 360 - 24.7;
    return Math.toRadians(degrees) + 3.05;
  }

  public void setGoal(double goal) {
    controller.setGoal(goal);
  }

  @Override
  public void periodic() {
    double pid = controller.calculate(getPosition());
    double ff = feedforward.calculate(controller.getGoal().position, 
      controller.getGoal().velocity);

    pivotMotor.setVoltage(pid + ff);    
  }
}
