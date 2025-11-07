package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import java.util.function.Function;

public class SuperstructureSetpoints2 {
  public static final double offset = 0.0555;

  public static final SuperstructureState IDLE =
      new SuperstructureState(
          Value.of(0.04), Rotation2d.fromDegrees(150), RotationsPerSecond.of(0.0));

  public static final SuperstructureState CORAL_STOW =
      new SuperstructureState(
          Value.of(0.04), Rotation2d.fromDegrees(150), RotationsPerSecond.of(0.0));

  public static final SuperstructureState HP =
      new SuperstructureState(
          Value.of(0.048), Rotation2d.fromDegrees(50), RotationsPerSecond.of(0));

  public static final SuperstructureState PROCESSOR =
      new SuperstructureState(
          Value.of(0.23), Rotation2d.fromDegrees(100), RotationsPerSecond.of(8));

  public static final SuperstructureState L4_PREP =
      new SuperstructureState(
          Value.of(0.90135 + offset), Rotation2d.fromDegrees(230), RotationsPerSecond.of(0));

  public static final SuperstructureState AUTO_L4_PREP =
      new SuperstructureState(
          Value.of(0.30), Rotation2d.fromDegrees(130), RotationsPerSecond.zero());

  public static final SuperstructureState L3_PREP =
      new SuperstructureState(
          Value.of(0.573 + offset * 0), Rotation2d.fromDegrees(220), RotationsPerSecond.of(0));

  public static final SuperstructureState L2_PREP =
      new SuperstructureState(
          Value.of(0.3693 + offset * 0), Rotation2d.fromDegrees(220), RotationsPerSecond.of(0));

  public static final SuperstructureState L1_PREP =
      new SuperstructureState(
          Value.of(0.1 + offset * 0), Rotation2d.fromDegrees(220), RotationsPerSecond.of(0));

  public static final SuperstructureState L1 =
      new SuperstructureState(
          Value.of(0.23 + offset * 0), Rotation2d.fromDegrees(220), RotationsPerSecond.of(-4));

  public static final SuperstructureState L2 =
      new SuperstructureState(
          Value.of(0.3693 + offset * 0), Rotation2d.fromDegrees(220), RotationsPerSecond.of(-5));

  public static final SuperstructureState L3 =
      new SuperstructureState(
          Value.of(0.573 + offset * 0), Rotation2d.fromDegrees(220), RotationsPerSecond.of(-5));

  public static final SuperstructureState L4 =
      new SuperstructureState(
          Value.of(0.90135 + offset), Rotation2d.fromDegrees(230), RotationsPerSecond.of(-5));

  public static final SuperstructureState L4_AUTO =
      new SuperstructureState(
          Value.of(0.90135 + offset), Rotation2d.fromDegrees(230), RotationsPerSecond.of(-20));

  public static final SuperstructureState L2_ALGAE =
      new SuperstructureState(
          Value.of(0.2), Rotation2d.fromDegrees(135), RotationsPerSecond.of(-10));

  public static final SuperstructureState L3_ALGAE =
      new SuperstructureState(
          Value.of(0.40), Rotation2d.fromDegrees(135), RotationsPerSecond.of(-10));

  public static final SuperstructureState ALGAE_STOW =
      new SuperstructureState(
          Value.of(0.12), Rotation2d.fromDegrees(135), RotationsPerSecond.of(-10));

  public static final SuperstructureState ALGAE_STOW_LOWER =
      new SuperstructureState(
          Value.of(0.32), Rotation2d.fromDegrees(135), RotationsPerSecond.of(-10));

  public static final SuperstructureState BARGE =
      new SuperstructureState(
          Value.of(0.95), Rotation2d.fromDegrees(110), RotationsPerSecond.of(0));

  public static final SuperstructureState BARGE_SCORE =
      new SuperstructureState(
          Value.of(0.95), Rotation2d.fromDegrees(110), RotationsPerSecond.of(10));

  public static final SuperstructureState BARGE_NO_OUTTAKE =
      new SuperstructureState(
          Value.of(0.95), Rotation2d.fromDegrees(150), RotationsPerSecond.of(3));

  public static final SuperstructureState GROUND_INTAKE =
      new SuperstructureState(
          Value.of(0.0), Rotation2d.fromDegrees(135), RotationsPerSecond.of(-9));

  public static final SuperstructureState HOMING =
      new SuperstructureState(Value.of(0.0), Rotation2d.fromDegrees(135), RotationsPerSecond.of(0));

  public static final SuperstructureState HP_LOWER =
      new SuperstructureState(
          Value.of(0.048), Rotation2d.fromDegrees(54), RotationsPerSecond.of(20));

  public static final SuperstructureState HP_LOWER_AUTO =
      new SuperstructureState(
          Value.of(0.048), Rotation2d.fromDegrees(54), RotationsPerSecond.of(8));

  public static final SuperstructureState HOMING_READY =
      new SuperstructureState(
          Value.of(0.1), Rotation2d.fromDegrees(135), RotationsPerSecond.zero());

  public static final SuperstructureState REJECT_CORAL =
      new SuperstructureState(Value.of(0), Rotation2d.fromDegrees(150), RotationsPerSecond.of(-5));

  public static final SuperstructureState REJECT_ALGAE =
      new SuperstructureState(Value.of(0), Rotation2d.fromDegrees(135), RotationsPerSecond.of(5));

  public abstract static class SuperstructureStateBase {
    public abstract Dimensionless getElevatorProportion();

    public abstract Rotation2d getWristRotation();

    public abstract AngularVelocity getWheelSpeed();
  }

  public static class SuperstructureState extends SuperstructureStateBase {
    private final Dimensionless elevatorProportion;
    private final Rotation2d wristRotation;
    private final AngularVelocity wheelSpeed;

    public SuperstructureState(
        Dimensionless elevatorProportion, Rotation2d wristRotation, AngularVelocity wheelSpeed) {
      this.elevatorProportion = elevatorProportion;
      this.wristRotation = wristRotation;
      this.wheelSpeed = wheelSpeed;
    }

    public Angle getElevatorEncoderPosition() {
      return ElevatorConstants.rotationsToPercentage.convertBackwards(this.getElevatorProportion());
    }

    public Angle getWristEncoderPosition() {
      return WristConstants.RotationToPosition.convert(wristRotation);
    }

    @Override
    public Dimensionless getElevatorProportion() {
        return elevatorProportion;
    }

    @Override
    public Rotation2d getWristRotation() {
        return wristRotation;
    }

    @Override
    public AngularVelocity getWheelSpeed() {
        return wheelSpeed;
    }
  }

  @SuppressWarnings("unused")
  public static interface UnitConverter<A, B> {
    /** Performs the forward unit conversion */
    B convert(A amountA);

    /** Performs the inverse unit conversion */
    A convertBackwards(B amountB);

    /** Equivalent to {@link UnitConverter#compose UnitConvertor.compose(this, next)}. */
    default <C> UnitConverter<A, C> then(UnitConverter<B, C> next) {
      return compose(this, next);
    }

    /** Equivalent to {@link UnitConverter#invert UnitConvertor.invert(this)}. */
    default UnitConverter<B, A> inverted() {
      return invert(this);
    }

    /**
     * Returns the inverse of a unit convertor, swapping {@link UnitConverter#convert} and {@link
     * UnitConverter#convertBackwards}
     */
    static <A, B> UnitConverter<B, A> invert(UnitConverter<A, B> convertor) {
      return new UnitConverter<>() {
        @Override
        public B convertBackwards(A amountB) {
          return convertor.convert(amountB);
        }

        @Override
        public A convert(B amountA) {
          return convertor.convertBackwards(amountA);
        }
      };
    }

    /** Composes unit convertors. */
    static <A, B, C> UnitConverter<A, C> compose(
        UnitConverter<A, B> convertor1, UnitConverter<B, C> convertor2) {
      return new UnitConverter<>() {
        @Override
        public C convert(A amountA) {
          return convertor2.convert(convertor1.convert(amountA));
        }

        @Override
        public A convertBackwards(C amountB) {
          return convertor1.convertBackwards(convertor2.convertBackwards(amountB));
        }
      };
    }

    /** Creates a unit converter between units that have a linear relationship */
    static UnitConverter<Double, Double> linear(double factor) {
      return linear(factor, 0, false);
    }

    /**
     * Creates a unit converter between units that have a linear relationship, with an offset
     *
     * @param applyOffsetFirst If true, {@link UnitConverter#convert convert(x)} returns (x +
     *     offset) * factor, otherwise, it returns (x * factor) + offset
     */
    static UnitConverter<Double, Double> linear(
        double factor, double offset, boolean applyOffsetFirst) {
      double actualOffset = applyOffsetFirst ? offset * factor : offset;

      return new UnitConverter<>() {
        @Override
        public Double convert(Double amountA) {
          return amountA * factor + actualOffset;
        }

        @Override
        public Double convertBackwards(Double amountB) {
          return (amountB - actualOffset) / factor;
        }
      };
    }

    /**
     * Creates a unit convertor that linearly scales a range (minA, maxA) to a range (minB, maxB).
     */
    static UnitConverter<Double, Double> linearConvertingRange(
        double minA, double maxA, double minB, double maxB) {
      double factor = (maxB - minB) / (maxA - minA);
      return linear(factor, minB - factor * minA, false);
    }

    static <A, B> UnitConverter<A, B> create(Function<A, B> forwards, Function<B, A> backwards) {
      return new UnitConverter<>() {
        @Override
        public B convert(A amountA) {
          return forwards.apply(amountA);
        }

        @Override
        public A convertBackwards(B amountB) {
          return backwards.apply(amountB);
        }
      };
    }

    static UnitConverter<Double, Distance> toDistance(DistanceUnit unit) {
      return create(unit::of, distance -> distance.in(unit));
    }

    static UnitConverter<Double, Rotation2d> radiansToRotation2d() {
      return create(Rotation2d::fromRadians, Rotation2d::getRadians);
    }
  }

  public static class WristConstants {
    public static final double HAS_CORAL_DEBOUNCE_SECONDS = 0.5;

    public static final Rotation2d MAX_ROTATION = Rotation2d.fromDegrees(230);
    public static final Rotation2d IDLE_ROTATION = Rotation2d.fromDegrees(30);

    public static final Angle MAX_POSITION = Rotations.of(0.075);
    public static final Angle MIN_POSITION = Rotations.of(0.988);

    public static final Rotation2d TRANSITION_ROTATION = Rotation2d.fromDegrees(145);

    public static final Current ALGAE_CURRENT = Amps.of(1);

    public static final Pair<Rotation2d, Rotation2d> SAFE_TRANSITION_RANGE_LOW =
        new Pair<>(Rotation2d.fromDegrees(145), Rotation2d.fromDegrees(110));

    public static final Pair<Rotation2d, Rotation2d> SAFE_TRANSITION_RANGE_HIGH =
        new Pair<>(MAX_ROTATION, Rotation2d.fromDegrees(95));

    public static final Pair<Rotation2d, Rotation2d> SAFE_TRANSITION_RANGE_INTERIOR =
        new Pair<>(IDLE_ROTATION, Rotation2d.fromDegrees(80));

    public static final UnitConverter<Rotation2d, Angle> RotationToPosition =
        UnitConverter.create(Rotation2d::getDegrees, Rotation2d::fromDegrees)
            .then(
                UnitConverter.linearConvertingRange(
                    IDLE_ROTATION.getDegrees(),
                    MAX_ROTATION.getDegrees(),
                    MIN_POSITION.baseUnitMagnitude(),
                    MAX_POSITION.baseUnitMagnitude()))
            .then(
                UnitConverter.create(
                    d -> Angle.ofBaseUnits(d, Degree.getBaseUnit()), Angle::baseUnitMagnitude));

    public static final Angle wristTolerance = Degrees.of(1);
  }

  public static class ElevatorConstants {
    public static final Distance HEIGHT_MAX = Inches.of(90);
    public static final Distance HEIGHT_MIN = Inches.of(15);

    public static final Angle MOTOR_MAX = Rotations.of(60.91);
    public static final Angle MOTOR_MIN = Rotations.of(0);

    public static final Dimensionless PROPORTION_MAX = Percent.of(100);
    public static final Dimensionless PROPORTION_MIN = Percent.of(0);

    private static final UnitConverter<Double, Angle> DoubleToRotations =
        UnitConverter.create(
            d -> Angle.ofBaseUnits(d, Rotations.getBaseUnit()), Angle::baseUnitMagnitude);

    private static final UnitConverter<Distance, Double> heightToDouble =
        UnitConverter.create(
            Distance::baseUnitMagnitude, d -> Distance.ofBaseUnits(d, Meter.one().baseUnit()));

    private static final UnitConverter<Double, Dimensionless> DoubleToPercentage =
        UnitConverter.create(
            d -> Dimensionless.ofBaseUnits(d, Value.getBaseUnit()),
            Dimensionless::baseUnitMagnitude);

    public static final UnitConverter<Distance, Angle> heightToRotations =
        heightToDouble
            .then(
                UnitConverter.linearConvertingRange(
                    HEIGHT_MIN.baseUnitMagnitude(),
                    HEIGHT_MAX.baseUnitMagnitude(),
                    MOTOR_MIN.baseUnitMagnitude(),
                    MOTOR_MAX.baseUnitMagnitude()))
            .then(DoubleToRotations);

    public static final UnitConverter<Distance, Dimensionless> heightToPercentage =
        heightToDouble
            .then(
                UnitConverter.linearConvertingRange(
                    HEIGHT_MIN.baseUnitMagnitude(),
                    HEIGHT_MAX.baseUnitMagnitude(),
                    PROPORTION_MIN.baseUnitMagnitude(),
                    PROPORTION_MAX.baseUnitMagnitude()))
            .then(DoubleToPercentage);

    public static final UnitConverter<Angle, Dimensionless> rotationsToPercentage =
        DoubleToRotations.inverted()
            .then(
                UnitConverter.linearConvertingRange(
                    MOTOR_MIN.baseUnitMagnitude(),
                    MOTOR_MAX.baseUnitMagnitude(),
                    PROPORTION_MIN.baseUnitMagnitude(),
                    PROPORTION_MAX.baseUnitMagnitude()))
            .then(DoubleToPercentage);

    public static final Dimensionless elevatorTolerance = Percent.of(0.5);

    /**
     * The point at which we can expand the wrist safe zone to include the full exterior range of
     * motion
     */
    public static final Dimensionless SAFE_ZONE_EXPANSION = Percent.of(25);
  }
}
