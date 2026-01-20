package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math.*;

public class Turret extends SubsystemBase {
  private static final double[] redHub = {4.74, 4.166};
  private static final double[] blueHub = {11.951, 4.166};
  @Logged public double redAngle;
  @Logged public double blueAngle;
  private CommandSwerveDrivetrain drive;

  public Turret(CommandSwerveDrivetrain drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    redTurretRotation();
    blueTurretRotation();
  }

  public void redTurretRotation() {
    redAngle =
        Math.toDegrees(
            Math.atan((drive.getPose().getX()) - redHub[0])
                / ((drive.getPose().getY()) - redHub[1]));
  }

  public void blueTurretRotation() {
    blueAngle =
        Math.toDegrees(
            Math.atan((drive.getPose().getX()) - blueHub[0])
                / ((drive.getPose().getY()) - blueHub[1]));
  }

  @Logged
  public Pose2d getBluePose() {
    return new Pose2d(drive.getPose().getTranslation(), new Rotation2d(Degrees.of((blueAngle))));
  }

  @Logged
  public Pose2d getRedPose() {
    return new Pose2d(drive.getPose().getTranslation(), new Rotation2d(Degrees.of((redAngle))));
  }
}
