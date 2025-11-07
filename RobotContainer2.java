// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.RollerCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(0);
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  RollerSubsystem rollerSubsystem = new RollerSubsystem();
  WristSubsystem wristSubsystem = new WristSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.rightTrigger().whileTrue(new ElevatorCommand(elevatorSubsystem, SuperstructureSetpoints2.L4_PREP.getElevatorEncoderPosition().in(Rotations)));
    controller.leftTrigger().whileTrue(new ElevatorCommand(elevatorSubsystem, SuperstructureSetpoints2.L3_PREP.getElevatorEncoderPosition().in(Rotations)));
    controller.leftBumper().whileTrue(new ElevatorCommand(elevatorSubsystem, SuperstructureSetpoints2.L2_PREP.getElevatorEncoderPosition().in(Rotations)));
    controller.rightBumper().whileTrue(new ElevatorCommand(elevatorSubsystem, SuperstructureSetpoints2.L1_PREP.getElevatorEncoderPosition().in(Rotations)));

    // elevatorSubsystem.setDefaultCommand(new ElevatorCommand(elevatorSubsystem, SuperstructureSetpoints2.L1_PREP.getElevatorEncoderPosition().in(Rotations)));
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
