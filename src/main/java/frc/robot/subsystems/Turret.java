package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math.*;

public class Turret extends SubsystemBase {
  // redHub = {11.93, 4.03};
  // blueHub = {4.63, 4.03};
  private CommandSwerveDrivetrain drive;

  Translation2d redHub = new Translation2d(11.91, 4.03);
  Translation2d blueHub = new Translation2d(4.63, 4.03);
  Translation2d hub;
  @Logged Rotation2d angle;
  @Logged Pose2d robotPose;

  public Turret(CommandSwerveDrivetrain drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    turretRotation();
  }

  public Translation2d gethub() {
    if (drive.getPose().getX() < 8.3) {
      hub = blueHub;
    }
    if (drive.getPose().getX() > 8.3) {
      hub = redHub;
    }
    return hub;
  }

  public void turretRotation() {
    angle = ((gethub().minus(drive.getPose().getTranslation())).getAngle());
  }

  @Logged
  public Pose2d getTurretPose() {
    return new Pose2d(drive.getPose().getTranslation(), angle);
  }
}
