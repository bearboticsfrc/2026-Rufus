package frc.robot.subsystems.fuelCalc;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class FuelCalculator {

private final CommandSwerveDrivetrain drivetrain;
private double offset = 60.0;
private final PhotonCamera centerCamera = new PhotonCamera("Fuel_OV9782_Center");
private final PhotonCamera rightCamera = new PhotonCamera("Fuel_OV9782_Right");
private final PhotonCamera leftCamera = new PhotonCamera("Fuel_OV9782_Left");

public List<PhotonTrackedTarget> targeter() {
    
    List<PhotonTrackedTarget> allTargets = new ArrayList<>();
    List<PhotonPipelineResult> results1 = centerCamera.getAllUnreadResults();
    List<PhotonPipelineResult> results2 = rightCamera.getAllUnreadResults();
    List<PhotonPipelineResult> results3 = leftCamera.getAllUnreadResults();
    if (!results2.isEmpty()) {

      PhotonPipelineResult rightResult = results2.get(results2.size() - 1);

      for (PhotonTrackedTarget target : rightResult.getTargets()) {
        PhotonTrackedTarget newTarget =
            new PhotonTrackedTarget(
                target.getYaw() + offset,
                target.getPitch(),
                target.getArea(),
                target.getSkew(),
                target.getFiducialId(),
                target.getDetectedObjectClassID(),
                target.getDetectedObjectConfidence(),
                target.getBestCameraToTarget(),
                target.getAlternateCameraToTarget(),
                target.getPoseAmbiguity(),
                target.getMinAreaRectCorners(),
                target.getDetectedCorners());
        allTargets.add(newTarget);
      }

      if (!results3.isEmpty()) {

        PhotonPipelineResult leftResult = results3.get(results3.size() - 1);

        for (PhotonTrackedTarget target : leftResult.getTargets()) {
          PhotonTrackedTarget newTarget =
              new PhotonTrackedTarget(
                  target.getYaw() - offset,
                  target.getPitch(),
                  target.getArea(),
                  target.getSkew(),
                  target.getFiducialId(),
                  target.getDetectedObjectClassID(),
                  target.getDetectedObjectConfidence(),
                  target.getBestCameraToTarget(),
                  target.getAlternateCameraToTarget(),
                  target.getPoseAmbiguity(),
                  target.getMinAreaRectCorners(),
                  target.getDetectedCorners());
          allTargets.add(newTarget);
        }
      }

      if (!results1.isEmpty()) {
        PhotonPipelineResult centerResult = results1.get(results1.size() - 1);
        allTargets.addAll(centerResult.getTargets());
      }
      if (allTargets.isEmpty()) {
        return allTargets;
      }
    }
    return allTargets;
  }

public PhotonTrackedTarget getPreferredTarget(List<PhotonTrackedTarget> results) {
    double closestPitch = 99;
    PhotonTrackedTarget bestTarget = null;
    for (PhotonTrackedTarget target : results) {
      if (target.getPitch() < closestPitch) {
        closestPitch = target.getPitch();
        bestTarget = target;
      }
    }
    return bestTarget;
  }

public PhotonTrackedTarget getMostTargets(List<PhotonTrackedTarget> results) {
    double closestPitch = 99;
    int leftCount = 0;
    int centerCount = 0;
    int rightCount = 0;
    PhotonTrackedTarget bestTarget = null;
    for (PhotonTrackedTarget target : results) {
      if (target.getYaw() > 30) {
        rightCount++;
      } else if (target.getYaw() < -30) {
        leftCount++;
      } else {
        centerCount++;
      }
    }
    if (rightCount > centerCount && rightCount > leftCount) {
      for (PhotonTrackedTarget target : results) {
        if (target.getPitch() < closestPitch && target.getYaw() > 30) {
          closestPitch = target.getPitch();
          bestTarget = target;
        }
      }
    } else if (centerCount < leftCount) {
      for (PhotonTrackedTarget target : results) {
        if (target.getPitch() < closestPitch && target.getYaw() < -30) {
          closestPitch = target.getPitch();
          bestTarget = target;
        }
      }
    } else {
      for (PhotonTrackedTarget target : results) {
        if (target.getPitch() < closestPitch && target.getYaw() >= -30 && target.getYaw() <= 30) {
          closestPitch = target.getPitch();
          bestTarget = target;
        }
      }
    }
    return bestTarget;
  }

@Logged
public Pose2d getTargetPose() {
      Transform2d targetCamera;
      PhotonTrackedTarget target = getPreferredTarget(targeter());

      // camera of target
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

      Pose2d cameraPose = drivetrain.getPose().transformBy(targetCamera);
      Translation2d fromCamera = new Translation2d(1 / Math.tan(Math.toRadians(22) + Math.toRadians(target.getPitch())), new Rotation2d(Degrees.of(target.getYaw())));
      Translation2d targetPose =
          cameraPose
              .getTranslation()
              .plus(fromCamera.rotateBy(cameraPose.getRotation())); // final translation
      Rotation2d rotationToTarget =
          cameraPose.getRotation().plus(new Rotation2d(Degrees.of(target.getYaw())));
      
      return new Pose2d(targetPose, rotationToTarget);
  }
}
