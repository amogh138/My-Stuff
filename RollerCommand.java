package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsystem;

public class RollerCommand extends Command {
  private final RollerSubsystem rollerSubsystem;
  private final double speed;

  public RollerCommand(RollerSubsystem rollerSubsystem,double speed) {
    this.rollerSubsystem = rollerSubsystem;
    this.speed = speed;
    addRequirements(rollerSubsystem);
  }
  @Override
  public void initialize() {
    rollerSubsystem.setMotorSpeed(0);
  } // once

  @Override
  public void execute() {
    rollerSubsystem.setMotorSpeed(speed);
  } 

  @Override
  public void end(boolean interrupted) {
    rollerSubsystem.setMotorSpeed(0);
  } 

  @Override
  public boolean isFinished() {
    return false;
  }
}
