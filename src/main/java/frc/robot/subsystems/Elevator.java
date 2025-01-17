// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final TalonFX leftMotor = new TalonFX(20, "canivore");
  private final TalonFX rightMotor = new TalonFX(21, "canivore");
  private final DigitalInput lowerLimitSwitch = new DigitalInput(0);
  private final CANcoder encoder = new CANcoder(22, "canivore");
  private boolean pidControllerEnabled = true;

  private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0, .15, 2.6);
  private final ProfiledPIDController controller =
    new ProfiledPIDController(
      3.9,
      0,
      0.001,
      new TrapezoidProfile.Constraints(5.2, 8)
  );

  /** Creates a new Elevator. */
  public Elevator() {
    controller.setTolerance(0.05);
    encoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)));
    controller.setGoal(0);
    leftMotor.setInverted(false);
    rightMotor.setInverted(true);

    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void moveElevatorManual(boolean isDown) {
    double speed = 0.7;
        if(isDown) {
            if(limitSwitchPressed()) {
                stopElevator();
                return;
            }
            
        }
        else {
            if(elevatorAtMax()) {
                stopElevator();
                return;
            }
            speed = -speed;
        }
        leftMotor.set(speed);
        rightMotor.set(speed);
  }

  public Command getHomeCommand() {
    return Commands.runOnce(this::disablePid)
    .andThen(
      Commands.run(
        () -> moveElevatorManual(true),
        this
      ).until(this::limitSwitchPressed)
    ).andThen(Commands.runOnce(this::stopElevator));
  }

  public Command getFullExtendCommand() {
    return new InstantCommand(() -> {
      setGoal(3);
      enablePid();
    });
  }

  public double getPosition() {
    return encoder.getPosition().getValue();
  }

  public boolean elevatorAtMax() {
    return getPosition() >= 3;
  }

  public boolean limitSwitchPressed() {
    return !lowerLimitSwitch.get();
  }

  private void enablePid() {
    pidControllerEnabled = true;
    controller.reset(getPosition());
  }

  private void disablePid() {
    pidControllerEnabled = false;
  }

  public void stopElevator() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void setGoal(double goal) {
    controller.setGoal(goal);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("elevator/lowerLimit", limitSwitchPressed());
    SmartDashboard.putBoolean("elevator/upperLimit", elevatorAtMax());
    SmartDashboard.putNumber("elevator/cancoderPosition", getPosition());
    SmartDashboard.putNumber("elevator/goal", controller.getGoal().position);
    SmartDashboard.putBoolean("elevator/pid", pidControllerEnabled);
    SmartDashboard.putNumber("elevator/volts", leftMotor.getMotorVoltage().getValue());
    if (pidControllerEnabled) {
      double feedforward = elevatorFeedforward.calculate(controller.getSetpoint().velocity);
      double pidResult = controller.calculate(getPosition());
      double volts = -(feedforward + pidResult);
      if ((limitSwitchPressed() && volts >= 0) || (elevatorAtMax() && volts <= 0)) {
        stopElevator();
        return;
      }
      leftMotor.setVoltage(volts);
      rightMotor.setVoltage(volts);
    }
  }
}
