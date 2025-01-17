// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.PodiumShot;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    //Declares/Creates new intake object
    public static Intake intake  = new Intake();
    public static Shooter shooter = new Shooter();
    public static Elevator elevator = new Elevator();
    public static AmpArm arm = new AmpArm();
    //Declares the controller
    private final CommandXboxController controller = new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        controller.rightBumper().whileTrue(new IntakeCommands());
        controller.leftBumper().whileTrue(new ShooterCommand());
        controller.a().onTrue(elevator.getHomeCommand());
        controller.y().onTrue(elevator.getFullExtendCommand());
        controller.povDown().onTrue(arm.getHomeCommand());
        controller.povUp().onTrue(arm.getAmpShootCommand());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
