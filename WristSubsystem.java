package frc.robot.subsystems;

import frc.robot.Constants;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * One SparkFlex, one {@link au.grapplerobotics.LaserCan} distance sensor. Figure this one out :)
 */
public class WristSubsystem extends SubsystemBase {
  private final SparkFlex wristMotor;
  private final LaserCan laserCan;

  public WristSubsystem() {
    wristMotor = new SparkFlex(Constants.IDs.WRIST, MotorType.kBrushless);

    wristMotor.configure(
        new SparkFlexConfig().apply(new AbsoluteEncoderConfig().zeroOffset(0.27)),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
    laserCan = new LaserCan(0);
    // you need to add other stuff here, but you need this line
    // so that I can check that you did the distance sensor right.
    SmartDashboard.putData(
        "LaserCan",
        new Sendable() {

          @Override
          public void initSendable(SendableBuilder builder) {
            builder.addBooleanProperty("Has Coral", () -> hasCoral(), null);
          }
        });
    SmartDashboard.putNumber("Wrist Encoder Reading", getAbsoluteEncoderMeasurement());
  }

  public void setMotorSpeed(double speed) {
    wristMotor.set(speed);
  }

  /**
   * We will say that the bot has coral if the LaserCan measurement is less than 23 mm.
   *
   * @return if the bot has coral or not.
   */
  public boolean hasCoral() {
    Measurement measurement = laserCan.getMeasurement();
    if (measurement == null) {
      return false; 
  }
    return measurement.distance_mm < 50;
  }

  public double getAbsoluteEncoderMeasurement(){
    return wristMotor.getAbsoluteEncoder().getPosition();
  }
}
