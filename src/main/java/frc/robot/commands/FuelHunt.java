
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

public enum TargetSide { LEFT, RIGHT, BOTH }

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

  @Override
  public void execute() {
  List<PhotonPipelineResult> results1 = centerCamera.getAllUnreadResults(); 
  List<PhotonPipelineResult> results2 = rightCamera.getAllUnreadResults(); 
  List<PhotonPipelineResult> results3 = leftCamera.getAllUnreadResults();

    List<PhotonPipelineResult> allResults = new ArrayList<>();
      allResults.addAll(results1);
      allResults.addAll(results2);
      allResults.addAll(results3);

    if (allResults.isEmpty()) {
      driveSubsystem.setControl(new SwerveRequest.Idle());
      return;
    }

    PhotonPipelineResult latestWithTarget = null;
    
    for (PhotonPipelineResult r : allResults) { 
      if (r.hasTargets()) 
      { latestWithTarget = r; } 
    }

    if (latestWithTarget == null) { 
      driveSubsystem.setControl(new SwerveRequest.Idle()); 
      return; 
    }

    PhotonTrackedTarget target = latestWithTarget.getBestTarget(); 

    double targetX = target.getPitch();
    double targetY = target.getYaw();

    if (targetX < 6) targetX = 0;

    double xSpeed = xSpeedController.calculate(targetX, 0);
    double rot = rotSpeedController.calculate(targetY, 0);

    drive(xSpeed, 0.0, rot);
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
