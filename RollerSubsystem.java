package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
  private final SparkFlex rollerMotor;

  public RollerSubsystem() {
    rollerMotor = new SparkFlex(Constants.IDs.ROLLER, MotorType.kBrushless);
  }

  public void setMotorSpeed(double speed) {
    rollerMotor.set(speed);
  }
}
