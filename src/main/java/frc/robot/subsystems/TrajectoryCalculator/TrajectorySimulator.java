package frc.robot.subsystems.TrajectoryCalculator;

// import java.util.List;
// import java.util.Scanner;

// /**
//  * Interactive trajectory simulator for the sphere projectile. Allows user input for velocity and
//  * launch angle.
//  */
public class TrajectorySimulator {

  // private static final double SPHERE_DIAMETER_INCHES = 6;
  // private static final double SPHERE_MASS_LBS = 0.5;
  // private static final double DEFAULT_INITIAL_HEIGHT = 5; // feet

  // public static void main(String[] args) {
  //   Scanner scanner = new Scanner(System.in);

  //   System.out.println("========================================");
  //   System.out.println("    Sphere Projectile Trajectory");
  //   System.out.println("    Simulator (7000 ft Elevation)");
  //   System.out.println("========================================");
  //   System.out.println();
  //   System.out.println("Projectile Specifications:");
  //   System.out.println("  Diameter: " + SPHERE_DIAMETER_INCHES + " inches");
  //   System.out.println("  Mass: " + SPHERE_MASS_LBS + " lbs");
  //   System.out.println("  Elevation: 7000 feet");
  //   System.out.println();

  //   try {
  //     // Get initial height
  //     System.out.print("Initial height (feet) [default: " + DEFAULT_INITIAL_HEIGHT + "]: ");
  //     String heightInput = scanner.nextLine().trim();
  //     double height =
  //         heightInput.isEmpty() ? DEFAULT_INITIAL_HEIGHT : Double.parseDouble(heightInput);

  //     if (height < 0) {
  //       System.out.println("Error: Height cannot be negative.");
  //       scanner.close();
  //       return;
  //     }

  //     // Get velocity
  //     System.out.print("Launch velocity (ft/s): ");
  //     double velocity = Double.parseDouble(scanner.nextLine().trim());

  //     if (velocity < 0) {
  //       System.out.println("Error: Velocity cannot be negative.");
  //       scanner.close();
  //       return;
  //     }

  //     // Get angle
  //     System.out.print("Launch angle (degrees): ");
  //     double angle = Double.parseDouble(scanner.nextLine().trim());

  //     if (angle < 0 || angle > 90) {
  //       System.out.println("Error: Angle must be between 0 and 90 degrees.");
  //       scanner.close();
  //       return;
  //     }

  //     System.out.println();
  //     System.out.println("Calculating trajectory...");
  //     System.out.println();

  //     // Create and calculate trajectory
  //     ProjectileTrajectory trajectory =
  //         ProjectileTrajectory.createSphere(
  //             height, velocity, angle, SPHERE_DIAMETER_INCHES, SPHERE_MASS_LBS);

  //     List<TrajectoryPoint> points = trajectory.calculateTrajectory();

  //     // Display results
  //     printResults(trajectory);

  //     // Ask if user wants detailed trajectory table
  //     System.out.println();
  //     System.out.print("Display detailed trajectory table? (y/n): ");
  //     String showTable = scanner.nextLine().trim().toLowerCase();

  //     if (showTable.equals("y") || showTable.equals("yes")) {
  //       System.out.println();
  //       printTrajectoryTable(points);
  //     }

  //   } catch (NumberFormatException e) {
  //     System.out.println("Error: Invalid input. Please enter valid numbers.");
  //   } finally {
  //     scanner.close();
  //   }
  // }

  // /** Print trajectory calculation results. */
  // private static void printResults(ProjectileTrajectory trajectory) {
  //   System.out.println("========================================");
  //   System.out.println("            RESULTS");
  //   System.out.println("========================================");
  //   System.out.println();
  //   System.out.println("Input Parameters:");
  //   System.out.println(
  //       "  Initial Height: " + String.format("%.2f", trajectory.getHeight()) + " ft");
  //   System.out.println(
  //       "  Launch Velocity: " + String.format("%.2f", trajectory.getSpeed()) + " ft/s");
  //   System.out.println(
  //       "  Launch Angle: " + String.format("%.2f", trajectory.getAngleDegrees()) + "°");
  //   System.out.println();
  //   System.out.println("Trajectory Metrics:");
  //   System.out.println(
  //       "  Maximum Height: " + String.format("%.2f", trajectory.getMaxHeight()) + " ft");
  //   System.out.println(
  //       "  Time to Max Height: " + String.format("%.3f", trajectory.getMaxHeightTime()) + " s");
  //   System.out.println(
  //       "  Range (Horizontal Distance): " + String.format("%.2f", trajectory.getRange()) + " ft");
  //   System.out.println(
  //       "  Total Flight Time: " + String.format("%.3f", trajectory.getImpactTime()) + " s");
  //   System.out.println();
  //   System.out.println("Impact Information:");
  //   System.out.println(
  //       "  Impact Speed: " + String.format("%.2f", trajectory.getImpactSpeed()) + " ft/s");

  //   double[] impactVel = trajectory.getImpactVelocity();
  //   System.out.println(
  //       "  Impact Velocity (Horizontal): " + String.format("%.2f", impactVel[0]) + " ft/s");
  //   System.out.println(
  //       "  Impact Velocity (Vertical): " + String.format("%.2f", impactVel[1]) + " ft/s");
  //   System.out.println(
  //       "  Impact Angle: "
  //           + String.format("%.2f", Math.toDegrees(Math.atan2(impactVel[1], impactVel[0])))
  //           + "°");
  //   System.out.println();
  // }

  // /** Print detailed trajectory table. */
  // private static void printTrajectoryTable(List<TrajectoryPoint> points) {
  //   System.out.println("========================================");
  //   System.out.println("       DETAILED TRAJECTORY TABLE");
  //   System.out.println("========================================");
  //   System.out.println();
  //   System.out.printf(
  //       "%-10s %-12s %-12s %-12s %-12s%n",
  //       "Time (s)", "Distance (ft)", "Height (ft)", "Horiz Vel", "Vert Vel");
  //   System.out.println("-".repeat(60));

  //   // Print every 10th point or if there are fewer than 50 points, print more frequently
  //   int step = Math.max(1, points.size() / 20);

  //   for (int i = 0; i < points.size(); i += step) {
  //     TrajectoryPoint point = points.get(i);
  //     System.out.printf(
  //         "%-10.3f %-12.2f %-12.2f %-12.2f %-12.2f%n",
  //         point.time, point.x, point.y, point.vx, point.vy);
  //   }

  //   // Always print the final point
  //   TrajectoryPoint finalPoint = points.get(points.size() - 1);
  //   if (points.size() % step != 0) {
  //     System.out.printf(
  //         "%-10.3f %-12.2f %-12.2f %-12.2f %-12.2f%n",
  //         finalPoint.time, finalPoint.x, finalPoint.y, finalPoint.vx, finalPoint.vy);
  //   }

  //   System.out.println();
  // }
}
