// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SimElevatorSubsystemCommand;
import frc.robot.subsystems.SimElevatorSubsystem;

public class RobotContainer {
  private final SimElevatorSubsystem simElevatorSubsystem = new SimElevatorSubsystem();

  private final CommandXboxController controller = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.a().whileTrue(new SimElevatorSubsystemCommand(simElevatorSubsystem, Inches.of(50)));
    controller.b().whileTrue(new SimElevatorSubsystemCommand(simElevatorSubsystem, Inches.of(90)));
    controller.x().whileTrue(new SimElevatorSubsystemCommand(simElevatorSubsystem, Inches.of(15)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
