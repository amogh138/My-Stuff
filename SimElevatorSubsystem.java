package frc.robot.subsystems;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimElevatorSubsystem extends SubsystemBase{
    private final ElevatorSim elevatorSim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(
            DCMotor.getKrakenX60(2), 
            10,
            0.0762,
            10)
            
        ,

        DCMotor.getKrakenX60(2), 
        Units.inchesToMeters(15), 
        Units.inchesToMeters(90), 
        true, 
        Units.inchesToMeters(15));;

    private final Mechanism2d elevatorMech = new Mechanism2d(120, 28.5);
    private final MechanismRoot2d elevatorMechRoot = elevatorMech.getRoot("Root", 28.5/2, 0);
    private final MechanismLigament2d elevatorMechLigament = elevatorMechRoot.append(
        new MechanismLigament2d("Elevator", getElevatorPosition().in(Meters), 90));

    public SimElevatorSubsystem() {
        
    }

    public Distance getElevatorPosition() {
        return Meters.of(elevatorSim.getPositionMeters());
    }

    public void setVoltage(Voltage voltage) {
        elevatorSim.setInputVoltage(voltage.in(Volts));
    }

    @Override
    public void periodic() {
        elevatorSim.update(0.02);
        SmartDashboard.putNumber("Elevator Position Meters", getElevatorPosition().in(Meters));

        elevatorMechLigament.setLength(
        getElevatorPosition().in(Meters));
        SmartDashboard.putData("Elevator",elevatorMech);
    }
}
