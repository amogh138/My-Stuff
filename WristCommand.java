package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends Command {
  private final WristSubsystem wristSubsystem;
  private final double position;
  private final static PIDController wristPIDController = new PIDController(0, 0, 0);

  public WristCommand(WristSubsystem wristSubsystem, double position) {
    this.wristSubsystem = wristSubsystem;
    this.position = position;
    SmartDashboard.putData(wristPIDController);
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {
    wristPIDController.reset();
    wristPIDController.setSetpoint(position);
  }

  @Override
  public void execute() {
    wristSubsystem.setMotorSpeed(wristPIDController.calculate(wristSubsystem.getAbsoluteEncoderMeasurement(), 0));
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setMotorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
