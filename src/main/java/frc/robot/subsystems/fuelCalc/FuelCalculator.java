package frc.robot.subsystems.fuelCalc;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class FuelCalculator {
  // supplier of fuel pose
  @Logged
public Supplier <Pose2d> getTargetPose(Supplier<PhotonTrackedTarget> targetSupplier, Supplier <Pose2d> driveSubsystem) {
    return () -> {
      Transform2d targetCamera;
      PhotonTrackedTarget target = targetSupplier.get();

      if (target == null || driveSubsystem.get() == null) return null;

      // finds camera of target
      if (target.getYaw() < -30) {
        targetCamera =
            new Transform2d(0, 0, new Rotation2d(Degrees.of(-70))); // left camera transform
      }
      if (target.getYaw() > 30) {
        targetCamera =
            new Transform2d(0, 0, new Rotation2d(Degrees.of(70))); // right camera transform
      } else {
        targetCamera =
            new Transform2d(0, 0, new Rotation2d(Degrees.of(0))); // center camera transform
      }

      // the field relative pose of the camera
      Pose2d cameraPose = driveSubsystem.get().transformBy(targetCamera);

      // distance and rotation of target from camera
      double distanceFromCamera =
          (1 / Math.tan(Math.toRadians(22) + Math.toRadians(target.getPitch())));
      Rotation2d rotationFromCamera = new Rotation2d(Degrees.of(target.getYaw()));
      Translation2d fromCamera = new Translation2d(distanceFromCamera, rotationFromCamera);

      // pose of target
      Translation2d targetPose =
          cameraPose
              .getTranslation()
              .plus(fromCamera.rotateBy(cameraPose.getRotation())); // final translation
      Rotation2d rotationToTarget =
          cameraPose.getRotation().plus(new Rotation2d(Degrees.of(target.getYaw())));
      return new Pose2d(targetPose, rotationToTarget);
    };
  }
}
