package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class FuelHunt extends Command {

  private final CommandSwerveDrivetrain driveSubsystem;
  private final PhotonCamera camera = new PhotonCamera("fueldetector");
  private final PIDController rotSpeedController = new PIDController(0.001, 0, 0);
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

  @Override
  public void execute() {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      drive(0, 0, 0);
      return;
    }
    PhotonPipelineResult result = results.get(results.size() - 1);
    if (!result.hasTargets()) {
      drive(0, 0, 0);
      return;
    }

    PhotonTrackedTarget target = result.getBestTarget();

    double targetY = target.getPitch();
    double targetX = target.getYaw();

    double xSpeed = -xSpeedController.calculate(targetY, 0);
    double rot = rotSpeedController.calculate(targetX, 0);

    if (xSpeedController.atSetpoint()) {
      xSpeed += 1;
    }

    drive(xSpeed, 0.0, rot);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive(0, 0, 0);
  }

  private void drive(double xSpeed, double ySpeed, double rot) {
    driveSubsystem.applyRequest(
        () -> drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rot));
  }
}
