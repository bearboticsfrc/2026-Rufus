package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import yams.units.EasyCRTConfig.CrtGearPair;

public class TurretAbsolutePosition {
  CANBus canbus = new CANBus("drive");

  private final CANcoder turretCANcoderMain = new CANcoder(31, canbus); // 10 tooth
  private final CANcoder turretCANcoderAux = new CANcoder(32, canbus); // 11 tooth

  private EasyCRT absoluteEncoder = new EasyCRT(getEasyCRTConfig());

  private EasyCRTConfig getEasyCRTConfig() {
    return new EasyCRTConfig(
            turretCANcoderMain.getAbsolutePosition().asSupplier(),
            turretCANcoderAux.getAbsolutePosition().asSupplier())
        .withCommonDriveGear(1, 93, 10, 11)
        .withCrtGearRecommendationInputs(93, 1.0)
        .withMechanismRange(Rotations.of(0.0), Rotations.of(1.0))
        .withMatchTolerance(Degrees.of(20))
        .withAbsoluteEncoder1Inverted(false)
        .withAbsoluteEncoder2Inverted(true)
        .withAbsoluteEncoderOffsets(Rotations.of(0), Rotations.of(0));
  }

  // @Logged
  public double getAbsoluteAngle() {
    return absoluteEncoder.getAngleOptional().orElse(Degrees.of(-1)).in(Degrees);
  }

  // @Logged
  public String getAbsoluteAngleStatus() {
    return absoluteEncoder.getLastStatus();
  }

  // @Logged
  public double getAbsoluteAngleIterations() {
    return absoluteEncoder.getLastIterations();
  }

  // @Logged
  public double getAbsoluteAngleMain() {
    return turretCANcoderMain.getAbsolutePosition().getValue().in(Degrees);
  }

  // @Logged
  public double getAbsoluteAngleAux() {
    return turretCANcoderAux.getAbsolutePosition().getValue().in(Degrees);
  }

  private static double GEAR_0_TOOTH_COUNT = 93.0;
  private static double GEAR_1_TOOTH_COUNT = 10.0;
  private static double GEAR_2_TOOTH_COUNT = 11.0;

  private static double SLOPE =
      (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
          / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * GEAR_0_TOOTH_COUNT);

  public static double calculateAngleFromEncoders(Angle encoder1, Angle encoder2) {
    double difference = encoder2.in(Degrees) - encoder1.in(Degrees);
    if (difference > 250) {
      difference -= 360;
    }
    if (difference < -250) {
      difference += 360;
    }
    difference *= SLOPE;
    double e1Rotations = (difference * GEAR_0_TOOTH_COUNT / GEAR_1_TOOTH_COUNT) / 360.0;
    double e1RotationsFloored = Math.floor(e1Rotations);
    double turretAngle =
        (e1RotationsFloored * 360 + encoder1.in(Degrees))
            * (GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT);
    if (turretAngle - difference < -100) {
      turretAngle += GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT * 360.0;
    } else if (turretAngle - difference > 100) {
      turretAngle -= GEAR_1_TOOTH_COUNT / GEAR_0_TOOTH_COUNT * 360.0;
    }

    return turretAngle;
  }

  // @Logged
  public double getAbsoluteAngle2910Style() {
    return calculateAngleFromEncoders(
        turretCANcoderMain.getAbsolutePosition().getValue(),
        turretCANcoderAux.getAbsolutePosition().getValue());
  }

  public static void main(String[] args) {

    CrtGearPair gearPair =
        EasyCRTConfig.findSmallestCrtGearPair(87, 1.0, Rotations.of(2), 1.2, 20, 50, 20);
    System.out.println("Pair = " + gearPair);
  }
}
