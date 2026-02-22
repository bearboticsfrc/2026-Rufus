package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class FuelHunt extends Command {

  public enum TargetSide {
    LEFT,
    RIGHT,
    BOTH
  }

  private TargetSide preferredSide = TargetSide.BOTH;

  /** Call to set the side to focus on. Default is BOTH. */
  public void setPreferredSide(TargetSide side) {
    this.preferredSide = side;
  }

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 4; // kSpeedAt12Volts desired top speed

  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final CommandSwerveDrivetrain driveSubsystem;
  private final PhotonCamera centerCamera = new PhotonCamera("Fuel_OV9782_Center");
  private final PhotonCamera rightCamera = new PhotonCamera("Fuel_OV9782_Right");
  private final PhotonCamera leftCamera = new PhotonCamera("Fuel_OV9782_Left");
  private final PIDController rotSpeedController = new PIDController(0.02, 0, 0);
  private final PIDController xSpeedController = new PIDController(0.1, 0, 0);

  private final SwerveRequest.RobotCentric drive =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public FuelHunt(CommandSwerveDrivetrain driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    addRequirements(driveSubsystem);
    rotSpeedController.setTolerance(2);
    xSpeedController.setTolerance(2);
  }

  @Override
  public void initialize() {}

  private double offset = 60.0;

  @Override
  public void execute() {
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
        driveSubsystem.setControl(new SwerveRequest.Idle());
        return;
      }

      PhotonTrackedTarget target = getMostTargets(allTargets);

      double targetX = target.getPitch();
      double targetY = target.getYaw();

      if (targetX < 6) targetX = 0;

      double xSpeed = xSpeedController.calculate(targetX, 0);
      double rot = rotSpeedController.calculate(targetY, 0);

      drive(xSpeed, 0.0, rot);
    }
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

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setControl(new SwerveRequest.Idle());
  }

  private void drive(double xSpeed, double ySpeed, double rot) {

    driveSubsystem.setControl(
        drive
            .withVelocityX(xSpeed * MaxSpeed)
            .withVelocityY(ySpeed * MaxSpeed)
            .withRotationalRate(rot * MaxAngularRate));
  }
}
