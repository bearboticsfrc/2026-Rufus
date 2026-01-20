package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.lang.Math.*;

public class Turret implements Subsystem {
  // coordinates of red and blue hubs
  private static final double[] redHub = {11.91, 4.03};
  private static final double[] blueHub = {4.62, 4.03};

  public static double redAngle;
  public static double blueAngle;
  private CommandSwerveDrivetrain drive;

  public Turret(CommandSwerveDrivetrain drive) {
    this.drive = drive;
  }

  double robotX = drive.getPose().getX();
  double robotY = drive.getPose().getY();

  @Override
  public void periodic() {
    // String team = DriverStation.getAlliance();
    redTurretRotation();

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
}
