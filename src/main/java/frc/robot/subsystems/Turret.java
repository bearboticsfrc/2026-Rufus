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
  @Logged Rotation2d robotRotation = drive.getPose().getRotation();
  @Logged Rotation2d turretRotation;
  @Logged Pose2d robotPose;
  @Logged Rotation2d turretRelativeRotation;

  public Turret(CommandSwerveDrivetrain drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    turretRotation();
  }

  public Translation2d gethub() {
    // adjusts which hub the turret rotates around based off the side of the field the robot is on
    if (drive.getPose().getX() < 8.3) {
      hub = blueHub;
    }
    if (drive.getPose().getX() > 8.3) {
      hub = redHub;
    }
    return hub;
  }

  // constantly gets the angle from the robot to the hub (turret rotation relative to hub)
  public void turretRotation() {
    turretRotation = ((gethub().minus(drive.getPose().getTranslation())).getAngle());
  }

  // turrets rotation relative to robot
  public Rotation2d turretRelativeRotation() {
    turretRelativeRotation = turretRotation.minus(robotRotation);
    return turretRelativeRotation;
  }

  //creates a new Pose2d that rotates around the hub
  @Logged
  public Pose2d getTurretPose() {
    return new Pose2d(drive.getPose().getTranslation(), turretRotation);
  }
}
