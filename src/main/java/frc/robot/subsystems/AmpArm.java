// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpArm extends SubsystemBase {
  private final TalonFX pivotMotor = new TalonFX(24, "canivore");
  private final TalonFX shootMotor = new TalonFX(23, "canivore");
  private final CANcoder encoder = new CANcoder(25, "canivore");

  private final ProfiledPIDController pivotController =
    new ProfiledPIDController(
      1.5, 
      0, 
      0,
      new Constraints(13,15)
  );

  private final ArmFeedforward pivotFeedforward = new ArmFeedforward(0.14,0.195,0.85);

  /** Creates a new AmpArm. */
  public AmpArm() {
    pivotController.enableContinuousInput(0, Math.PI * 2);
    pivotController.setTolerance(0.01);
    pivotController.setIZone(.1);

    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    shootMotor.setNeutralMode(NeutralModeValue.Coast);
    pivotMotor.setPosition(-90 / 14.7);
    setGoal(-2.4);
  }

  private double convert360To180(double angle) {
    return (angle + 180) % 360 - 180;
  }

  public double getMeasurementRadians() {
      double preConversion = convert360To180(((getCANCoderPositionDegrees())) % 360) * Math.PI / 180;
      return preConversion + 3.425 - Math.PI;
  }

  public double getCANCoderPositionDegrees() {
      return ((encoder.getAbsolutePosition().getValueAsDouble() * 360 + 360) - 63) % 360;
  }
  
  public void setGoal(double goal) {
    pivotController.setGoal(goal);
  }

  public Command getHomeCommand() {
    return new InstantCommand(() -> {
      setGoal(-2.4);
    }, this);
  }

  public Command getAmpShootCommand() {
    return new InstantCommand(() -> {
      setGoal(-0.55);
    }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pidOutput = pivotController.calculate(getMeasurementRadians());
    double feedForward = pivotFeedforward.calculate(pivotController.getSetpoint().position, pivotController.getSetpoint().velocity);
    double outputVoltage = pidOutput + feedForward;
    pivotMotor.setVoltage(outputVoltage);

    SmartDashboard.putNumber("arm/cancoderRadians", getMeasurementRadians());
    SmartDashboard.putNumber("arm/motorVoltage", outputVoltage);
    SmartDashboard.putNumber("arm/goal", pivotController.getGoal().position);
  }
}
