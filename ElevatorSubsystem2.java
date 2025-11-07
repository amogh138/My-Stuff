package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    private TalonFX leftMotor;
    private TalonFX rightMotor;
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
        false, 
        Units.inchesToMeters(15));

    private final Mechanism2d elevatorMech = new Mechanism2d(28.5, 120);
    private final MechanismRoot2d elevatorMechRoot = elevatorMech.getRoot("Root",28.5/2 , 0);
    private final MechanismLigament2d elevatorMechLigament = elevatorMechRoot.append(
        new MechanismLigament2d("Elevator", getElevatorPosition().in(Inches), 90)
    );


    public ElevatorSubsystem(){
        leftMotor = new TalonFX(Constants.IDs.ELEVATOR_LEFT,"MainCANivore");
        rightMotor = new TalonFX(Constants.IDs.ELEVATOR_RIGHT,"MainCANivore");

        var configuration = new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                ).withSlot0(
                    new Slot0Configs()
                //         .withKP(3.596)
                //         .withKI(0)
                //         .withKD(0)
            //      .withKG(2)
                //         .withKS(0.27919)
                //         .withKV(0.11358)
                );
        leftMotor.getConfigurator().apply(configuration);
        rightMotor.getConfigurator().apply(configuration);

        leftMotor.setControl(new Follower(Constants.IDs.ELEVATOR_RIGHT, true));

        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
        SmartDashboard.putNumber("Elevator Encoder Reading", getEncoderMeasurement());
    }

    public void setPosition(double speed){
        rightMotor.set(speed);
        elevatorSim.setInputVoltage(speed*12);
    }

    public double getEncoderMeasurement(){
        return rightMotor.getPosition().getValueAsDouble();
    }

    public Distance getElevatorPosition(){
        return Meters.of(elevatorSim.getPositionMeters());
    }

    public void stopMotor(){
        rightMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Encoder", getEncoderMeasurement());
        // elevatorSim.update(0.02);
        // SmartDashboard.putNumber("Elevator Position Meters", getElevatorPosition().in(Meters));

        // elevatorMechLigament.setLength(
        //     getElevatorPosition().in(Inches)
        // );

        // SmartDashboard.putData("Elevator", elevatorMech);
        SmartDashboard.putNumber("Elevator Current", rightMotor.getTorqueCurrent().getValueAsDouble());
    }
}
