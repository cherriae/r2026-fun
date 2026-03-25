package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

@Logged
public class Shooting {
  @Logged(name = "Target")
  private Pose2d target = Pose2d.kZero;

  @Logged(name = "VirtualTarget")
  private Pose2d virtualTarget = Pose2d.kZero;

  @Logged(name = "Heading")
  private Rotation2d shotHeading = Rotation2d.kZero;

  @Logged(name = "Newton Iterations")
  private int newtonIterations = 1;

  @Logged(name = "isValid")
  public boolean isValid = false;

  @Logged(name = "Convergence Failed")
  private boolean convergenceFailed = true;

  @Logged(name = "Coupling Degrees")
  private Angle couplingDegrees = Degrees.of(0);

  private final int maxIterations = 10;
  private final double errorTolerance = 0.5;

  public Shooting() {
    isValid = true;
  }

  public Shooting(
      boolean shooterReady,
      boolean headingWithinTolerance, // fix later with actual heading check
      Pose2d robotPose) {
    isValid =
        shooterReady
        && headingWithinTolerance
        && FieldUtil.inAllianceZone(robotPose)
        && !convergenceFailed;
  }

  // newton's method to find the heading to shoot at to hit the target
  public void calculateShotHeading(Pose2d robotPose, ChassisSpeeds robotSpeeds, Alliance alliance) {
    if (alliance == null) {
        return;
    }

    setTarget(
        alliance == Alliance.Red
            ? Constants.FieldConstants.redHub
            : Constants.FieldConstants.blueHub);

    Translation2d robotVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
    Translation2d robotTranslationToTarget =
        target.getTranslation().minus(robotPose.getTranslation());

    double initialGuess =
        robotTranslationToTarget.getNorm()
            / (robotTranslationToTarget.dot(robotVelocity) / robotTranslationToTarget.getNorm()
                + Constants.ShooterConstants.horizontolProjectileVelocity.in(MetersPerSecond));
    // (distance to target) / ((speed of how fast the robot is moving) + projectile horizontal
    // speed)
    // essentially: distance/(relative projectile speed) = time

    // iterations
    for (int i = 0; i < maxIterations; i++) {
      robotTranslationToTarget =
          target
              .getTranslation()
              .minus(robotVelocity.times(initialGuess)); // get position using the time computed

      double tof =
          Constants.ShooterConstants.hubTOF.get(
              robotTranslationToTarget.getNorm()); // interpolation of the tof
      double dT_dt =
          -robotTranslationToTarget.dot(robotVelocity)
              / (Constants.ShooterConstants.horizontolProjectileVelocity.in(MetersPerSecond)
                  * robotTranslationToTarget
                      .getNorm()); // need to fix when distance is outside table bounds

      double error = initialGuess - tof;
      double dError_dt = 1 - error/dT_dt;

      if (Math.abs(error) < errorTolerance) {
        couplingDegrees =
            Degrees.of(
                Math.toDegrees(
                    Math.acos(
                        Math.abs(
                            robotTranslationToTarget.dot(robotVelocity)
                                / (robotTranslationToTarget.getNorm()
                                    * robotVelocity.getNorm())))));

        setVirtualTarget(target.getTranslation().minus(robotVelocity.times(initialGuess)));
        double distanceToVirtualTarget =
            virtualTarget
                .getTranslation()
                .getDistance(robotPose.getTranslation()); // Get the distance to v target from robot
        Constants.ShooterConstants.hubPresets.get(
            distanceToVirtualTarget); // interpolate given distance

        setShotHeading(
            virtualTarget
                .getTranslation()
                .minus(robotPose.getTranslation())
                .getAngle()); // get shot heading given robot angle and v target angle

        newtonIterations = i + 1;
        convergenceFailed = false;
        break;
      }

      initialGuess = initialGuess - (error / dError_dt);
    }
    if (newtonIterations == maxIterations) {
        convergenceFailed = true;
    }
  }

  public Rotation2d getShotHeading() {
    return shotHeading;
  }

  public Pose2d getTarget() {
    return target;
  }

  public Pose2d getVirtualTarget() {
    return virtualTarget;
  }

  public void setShotHeading(Rotation2d _shotHeading) {
    shotHeading = _shotHeading;
  }

  public void setTarget(Translation2d _target) {
    target = new Pose2d(_target, Rotation2d.kZero);
  }

  public void setVirtualTarget(Translation2d _virtualTarget) {
    virtualTarget = new Pose2d(_virtualTarget, Rotation2d.kZero);
  }
}
