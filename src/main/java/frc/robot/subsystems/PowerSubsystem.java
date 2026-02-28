package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerSubsystem extends SubsystemBase {

  @Logged PowerDistribution examplePD = new PowerDistribution(1, ModuleType.kRev);

  public PowerSubsystem() {}

  @Override
  public void periodic() {}

  @Logged
  public double getChannel4Current() {
    return examplePD.getCurrent(4);
  }

  @Logged
  public double getChannel6Current() {
    return examplePD.getCurrent(6);
  }
}
