package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math.*;

public class Turret extends SubsystemBase {
  // coordinates of red and blue hubs
  private static final double[] redHub = {4.62, 4.03};
  private static final double[] blueHub = {11.91, 4.03};
  @Logged public double redAngle;
  @Logged public double blueAngle;
  private CommandSwerveDrivetrain drive;
  @Logged public double robotX;
  @Logged public double robotY;

  public Turret(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    robotX = drive.getPose().getX();
    robotY = drive.getPose().getY();
  }

  @Override
  public void periodic() {
    robotX = drive.getPose().getX();
    robotY = drive.getPose().getY();

    // String team = DriverStation.getAlliance();
    redTurretRotation();
    blueTurretRotation();
    Transform2d robot_to_turret_red =
        new Transform2d(new Translation2d(0, 0), new Rotation2d(Degrees.of(redAngle)));

    Transform2d robot_to_turret_blue =
        new Transform2d(new Translation2d(0, 0), new Rotation2d(Degrees.of(blueAngle)));
  }

  public void redTurretRotation() {
    redAngle = Math.atan((redHub[0] - robotX) / (redHub[1] - robotY));
    redAngle = redAngle * (180 / Math.PI);
  }

  public void blueTurretRotation() {
    blueAngle = Math.atan((blueHub[0] - robotX) / (blueHub[1] - robotY));
    blueAngle = blueAngle * (180 / Math.PI);
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
