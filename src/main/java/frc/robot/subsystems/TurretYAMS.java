package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretYAMS extends SubsystemBase {
  CANBus canbus = new CANBus("drive");
  private final TalonFX turretMotor = new TalonFX(30, canbus);

  private final CANcoder turretCANcoderMain = new CANcoder(31, canbus); // 10 tooth
  private final CANcoder turretCANcoderAux = new CANcoder(32, canbus); // 11 tooth
  public static double minRotations = -.45;
  public static double maxRotations = .45;

  public double getMinRotations() {
    return minRotations;
  }

  public double getMaxRotations() {
    return maxRotations;
  }

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              25, 0, .8981, RotationsPerSecond.of(20), RotationsPerSecondPerSecond.of(10))
          .withFeedforward(new SimpleMotorFeedforward(.38778, 2.3767, .077265))
          // Configure Motor and Mechanism properties
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(23.25))) // 12-30-10-93
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          // Setup Telemetry
          .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
          // Power Optimization
          .withStatorCurrentLimit(Amps.of(40))
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withSoftLimit(Rotations.of(getMinRotations()), Rotations.of(getMaxRotations()))
          .withOpenLoopRampRate(Seconds.of(0.25));
  private final SmartMotorController turretSMC =
      new TalonFXWrapper(turretMotor, DCMotor.getKrakenX44Foc(1), motorConfig);

  private final PivotConfig turretConfig =
      new PivotConfig(turretSMC)
          .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
          .withHardLimit(Degrees.of(-90), Degrees.of(90)) // Hard limit bc wiring prevents infinite
          // spinning
          .withSoftLimits(Rotations.of(getMinRotations()), Rotations.of(getMaxRotations()))
          .withTelemetry("TurretMech", TelemetryVerbosity.HIGH) // Telemetry
          .withMOI(Meters.of(0.25), Pounds.of(4)); // MOI Calculation

  private final Pivot turret = new Pivot(turretConfig);

  private EasyCRT absoluteEncoder = new EasyCRT(getEasyCRTConfig());

  private EasyCRTConfig getEasyCRTConfig() {
    return new EasyCRTConfig(
            turretCANcoderMain.getAbsolutePosition().asSupplier(),
            turretCANcoderAux.getAbsolutePosition().asSupplier())
        .withCommonDriveGear(1, 93, 10, 11)
        .withCrtGearRecommendationInputs(93, 1.0)
        .withMechanismRange(Rotations.of(0.0), Rotations.of(1.0))
        .withMatchTolerance(Degrees.of(20))
        .withAbsoluteEncoder1Inverted(true)
        .withAbsoluteEncoder2Inverted(false)
        .withAbsoluteEncoderOffsets(Rotations.of(-.343018), Rotations.of(-.451904));
  }

  private final Supplier<Pose2d> poseSupplier;

  public TurretYAMS(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  Translation2d blueHub = new Translation2d(4.63, 4.03);
  Rotation2d robotRotation;
  Rotation2d rotationToHub;
  @Logged Angle turretRelativeRotation;

  public Translation2d getHub() {
    return FlippingUtil.flipFieldPosition(blueHub);
  }

  // constantly gets the angle from the robot to the hub (turret rotation relative to hub)
  public void updateTurretRotation() {
    robotRotation = poseSupplier.get().getRotation();
    rotationToHub = ((getHub().minus(poseSupplier.get().getTranslation())).getAngle());
    turretRelativeRotation = Degrees.of(robotRotation.minus(rotationToHub).getDegrees());
    turretRelativeRotation =
        (turretRelativeRotation.gt(Degrees.of(180))
            ? turretRelativeRotation.minus(Degrees.of(180))
            : turretRelativeRotation);
  }

  // gets the distance to the hub
  @Logged
  public double getHubDistance() {
    return ((poseSupplier.get().getTranslation()).getDistance(getHub()));
  }

  @Logged
  public double getRobotRotationDegrees() {
    return robotRotation.getDegrees();
  }

  @Logged
  public double getTurretRelativeRotationDegrees() {
    return turretRelativeRotation.in(Degrees);
  }

  // turrets rotation relative to robot
  public Angle turretRelativeRotation() {
    return turretRelativeRotation;
  }

  public double getAbsoluteAngle() {
    return absoluteEncoder.getAngleOptional().orElse(Degrees.of(-1)).in(Degrees);
  }

  @Logged
  public String getAbsoluteAngleStatus() {
    return absoluteEncoder.getLastStatus();
  }

  @Logged
  public double getAbsoluteAngleIterations() {
    return absoluteEncoder.getLastIterations();
  }

  public boolean isAtAngle() {
    return turretSMC
        .getMechanismPosition()
        .isNear(turretSMC.getMechanismPositionSetpoint().orElse(Degrees.of(0)), Degrees.of(2.0));
  }

  @Logged
  public double getAbsoluteAngleMain() {
    return turretCANcoderMain.getAbsolutePosition().getValue().in(Degrees);
  }

  @Logged
  public double getAbsoluteAngleAux() {
    return turretCANcoderAux.getAbsolutePosition().getValue().in(Degrees);
  }

  private double wrapDegreesToSoftLimits(double targetDegrees) {

    double minDeg = getMinRotations() * 360.0;
    double maxDeg = getMaxRotations() * 360.0;
    double currentDeg = getAngle().in(Degrees);

    // Solve for integer n such that minDeg <= targetDegrees + 360*n <= maxDeg
    int nMin = (int) Math.ceil((minDeg - targetDegrees) / 360.0);
    int nMax = (int) Math.floor((maxDeg - targetDegrees) / 360.0);

    if (nMin <= nMax) {
      // At least one equivalent fits in soft limits.
      int nClosest = (int) Math.round((currentDeg - targetDegrees) / 360.0);
      int n =
          Math.max(nMin, Math.min(nClosest, nMax)); // clamp the closest candidate to allowed range
      return targetDegrees + n * 360.0;
    } else {
      // No equivalent fits in soft limits -> clamp to nearest soft limit endpoint.
      double toMin = Math.abs(currentDeg - minDeg);
      double toMax = Math.abs(currentDeg - maxDeg);
      return (toMin < toMax) ? minDeg : maxDeg;
    }
  }

  public Command setAngle(Angle angle) {
    double angleDegrees = wrapDegreesToSoftLimits(angle.in(Degrees));
    return turret.setAngle(Degrees.of(angleDegrees));
  }

  public void setAngleDirect(Angle angle) {
    turretSMC.setPosition(angle);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return turret.setAngle(angleSupplier);
  }

  public Angle getAngle() {
    return turret.getAngle();
  }

  public Command sysId() {
    return turret.sysId(
        Volts.of(4.0), // maximumVoltage
        Volts.per(Second).of(0.5), // step
        Seconds.of(8.0) // duration
        );
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return turret.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  @Override
  public void periodic() {
    updateTurretRotation();
    turret.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }

  // SysId routines
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> turretMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
              null,
              this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
