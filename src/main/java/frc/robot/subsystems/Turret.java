package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.spectrumLib.CachedDouble;
import frc.spectrumLib.util.Conversions;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;

public class Turret extends SubsystemBase implements NTSendable {

  private final CANBus canivore = new CANBus("drive");

  private final TalonFX motor = new TalonFX(30, canivore);

  /* Turret config values */
  @Getter private double currentLimit = 44;
  @Getter private double torqueCurrentLimit = 200;
  @Getter private double velocityKp = 23.221;
  @Getter private double velocityKi = 0.0;
  @Getter private double velocityKd = 0.8981;

  @Getter private double velocityKs = .38778;
  @Getter private double velocityKv = 2.3767;
  @Getter private double velocityKa = .077265;

  @Getter private double gearRatio = 23.25;

  @Getter private double statorLimit = 40;

  private final CachedDouble cachedRotations;
  private final CachedDouble cachedVoltage;
  private final CachedDouble cachedDegrees;
  private final CachedDouble cachedVelocity;
  private final CachedDouble cachedCurrent;

  //   public TurretConfig() {
  //     super("Turret", 5, Rio.CANIVORE);
  //     configPIDGains(0, velocityKp, 0, 0);
  //     configFeedForwardGains(velocityKs, velocityKv, 0, 0);
  //     configGearRatio(1);
  //     configSupplyCurrentLimit(currentLimit, true);
  //     configStatorCurrentLimit(torqueCurrentLimit, true);
  //     configForwardTorqueCurrentLimit(torqueCurrentLimit);
  //     configReverseTorqueCurrentLimit(torqueCurrentLimit);
  //     configNeutralBrakeMode(true);
  //     configCounterClockwise_Positive();
  //     setAttached(true);
  //   }
  // }

  private Rotation2d angleGoal;
  @Setter @Getter private boolean active;

  public Turret() {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kP = velocityKp;
    config.Slot0.kI = velocityKi;
    config.Slot0.kD = velocityKd;

    config.Slot0.kA = velocityKa;
    config.Slot0.kS = velocityKs;
    config.Slot0.kV = velocityKv;

    config.Feedback.RotorToSensorRatio = 1.0;
    config.Feedback.SensorToMechanismRatio = gearRatio;

    config.CurrentLimits.StatorCurrentLimit = statorLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    StatusCode status = motor.getConfigurator().apply(config);

    for (int i = 0; i < 2; i++) {
      if (status.isOK()) break;
      status = motor.getConfigurator().apply(config);
    }

    cachedCurrent = new CachedDouble(this::updateCurrent);
    cachedVoltage = new CachedDouble(this::updateVoltage);
    cachedRotations = new CachedDouble(this::updatePositionRotations);
    cachedDegrees = new CachedDouble(this::updatePositionDegrees);
    cachedVelocity = new CachedDouble(this::updateVelocityRPM);

    if (!status.isOK()) {
      System.out.println("ERROR Configuring Turret motor: " + status);
    }

    System.out.println("Turret Subsystem Initialized");
  }

  @Override
  public void periodic() {
    if (isActive()) {}
  }

  @Override
  public void initSendable(NTSendableBuilder builder) {
    builder.addStringProperty("CurrentCommand", this::getCurrentCommandName, null);
    builder.addDoubleProperty("Motor Voltage", this::getVoltage, null);
    // builder.addDoubleProperty("Rotations", this::getPositionRotations, null);
    builder.addDoubleProperty("Velocity RPM", this::getVelocityRPM, null);
    builder.addDoubleProperty("StatorCurrent", this::getStatorCurrent, null);
  }

  public void setTurretAngle(Rotation2d angle) {
    this.angleGoal = angle;
  }

  protected String getCurrentCommandName() {
    Command currentCommand = this.getCurrentCommand();
    if (currentCommand != null) {
      return currentCommand.getName();
    }

    return "none";
  }

  public double getVoltage() {
    return cachedVoltage.getAsDouble();
  }

  public double updateVoltage() {
    return motor.getMotorVoltage().getValueAsDouble();
  }

  /**
   * Update the value of the stator current for the motor
   *
   * @return
   */
  public double updateCurrent() {
    return motor.getStatorCurrent().getValueAsDouble();
  }

  /**
   * Updates the position of the motor
   *
   * @return motor position in rotations
   */
  private double updatePositionRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  public double getPositionRotations() {
    return cachedRotations.getAsDouble();
  }

  public double getVelocityRPM() {
    return cachedVelocity.getAsDouble();
  }

  public double getStatorCurrent() {
    return cachedCurrent.getAsDouble();
  }

  private double updatePositionDegrees() {
    return rotationsToDegrees(this::getPositionRotations);
  }

  /**
   * Rotations to Degrees
   *
   * @param rotations
   * @return degrees
   */
  public double rotationsToDegrees(DoubleSupplier rotations) {
    return 360 * rotations.getAsDouble();
  }

  // Get Velocity in RPM
  private double updateVelocityRPM() {
    return Conversions.RPStoRPM(updateVelocityRPS());
  }

  /**
   * Updates the velocity of the motor
   *
   * @return motor velocity in rotations/sec which are the CTRE native units
   */
  private double updateVelocityRPS() {
    return motor.getVelocity().getValueAsDouble();
  }
}
