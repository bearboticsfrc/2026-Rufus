package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NTSendableBuilder;
import frc.spectrumLib.Rio;
import frc.spectrumLib.mechanism.Mechanism;
import lombok.Getter;
import lombok.Setter;

public class Turret extends Mechanism {

  public static class TurretConfig extends Config {

    /* Turret config values */
    @Getter private double currentLimit = 44;
    @Getter private double torqueCurrentLimit = 200;
    @Getter private double velocityKp = .5;
    @Getter private double velocityKv = 0.2;
    @Getter private double velocityKs = .9;

    /* Sim Configs */
    @Getter private double intakeX = 0.8; // relative to elbow at 0 degrees
    @Getter private double intakeY = 1.3; // relative to elbow at 0 degrees
    @Getter private double wheelDiameter = 5.0;

    public TurretConfig() {
      super("Turret", 5, Rio.CANIVORE);
      configPIDGains(0, velocityKp, 0, 0);
      configFeedForwardGains(velocityKs, velocityKv, 0, 0);
      configGearRatio(1);
      configSupplyCurrentLimit(currentLimit, true);
      configStatorCurrentLimit(torqueCurrentLimit, true);
      configForwardTorqueCurrentLimit(torqueCurrentLimit);
      configReverseTorqueCurrentLimit(torqueCurrentLimit);
      configNeutralBrakeMode(true);
      configCounterClockwise_Positive();
      setAttached(true);
      // configForwardSoftLimit(170, true);
      // configReverseSoftLimit(-170, true);
    }
  }

  private TurretConfig config = new TurretConfig();

  private Rotation2d angleGoal;
  @Setter @Getter private boolean active;

  public Turret(TurretConfig config) {
    super(config);
    this.config = config;

    System.out.println(getName() + " Subsystem Initialized");
  }

  @Override
  public void periodic() {
    if (isActive()) {}
  }

  @Override
  public void setupStates() {}

  @Override
  public void setupDefaultCommand() {}

  @Override
  public void initSendable(NTSendableBuilder builder) {
    if (isAttached()) {
      builder.addStringProperty("CurrentCommand", this::getCurrentCommandName, null);
      builder.addDoubleProperty("Motor Voltage", this::getVoltage, null);
      // builder.addDoubleProperty("Rotations", this::getPositionRotations, null);
      builder.addDoubleProperty("Velocity RPM", this::getVelocityRPM, null);
      builder.addDoubleProperty("StatorCurrent", this::getStatorCurrent, null);
    }
  }

  public void setTurretAngle(Rotation2d angle) {
    this.angleGoal = angle;
  }
}
