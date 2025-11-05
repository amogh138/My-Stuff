package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SimElevatorSubsystem;
public class SimElevatorSubsystemCommand extends Command{

    private final SimElevatorSubsystem subsystem;
    private static PIDController controller = new PIDController(0, 0, 0);
    private final Distance setpoint;

    static{
        SmartDashboard.putData("PID Controller", controller);
    }

    public SimElevatorSubsystemCommand(SimElevatorSubsystem subsystem, Distance setpoint) {
        this.subsystem = subsystem;
        controller = new PIDController(0, 0, 0);
        this.setpoint = setpoint;
        addRequirements(subsystem);

        SmartDashboard.putData("PID Controller", controller);
    }

    @Override
    public void execute() {
        subsystem.setVoltage(Volts.of(controller.calculate(subsystem.getElevatorPosition().in(Meters), setpoint.in(Meters))));
    }
}
