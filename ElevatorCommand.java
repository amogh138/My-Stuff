package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final double position;
  private final static PIDController elevatorPidController = new PIDController(0,0,0);

  public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double position) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.position = position;
    addRequirements(elevatorSubsystem);
    SmartDashboard.putData(elevatorPidController);
  }

  @Override
  public void initialize() {
    elevatorPidController.reset();
    elevatorPidController.setSetpoint(position);
  }

  @Override
  public void execute(){
    elevatorSubsystem.setPosition(0.2 + elevatorPidController.calculate(elevatorSubsystem.getEncoderMeasurement()));
    SmartDashboard.putNumber("PID Output", elevatorPidController.calculate(elevatorSubsystem.getEncoderMeasurement()));
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
