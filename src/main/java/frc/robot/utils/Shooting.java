package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.LED;
import java.util.function.Supplier;

@Logged
public class Shooting {
  @Logged(name = "Flywheel Speed")
  public final MutAngularVelocity _flywheelSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Roller Speed")
  public final MutAngularVelocity _rollerSpeed = RotationsPerSecond.mutable(0);

  @Logged(name = "Floor Speed")
  public final MutAngularVelocity _floorSpeed = RotationsPerSecond.mutable(0);

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

  @Logged(name = "In Bound")
  public boolean inBound = false;

  @Logged(name = "Convergence Failed")
  private boolean convergenceFailed = true;

  @Logged(name = "Coupling Degrees")
  private Angle couplingDegrees = Degrees.of(0);

  private final int maxIterations = 10;
  private final double errorTolerance = 0.5;
  private final LED _led;

  public Shooting(LED led) {
    _led = led;

    new Trigger(() -> isValid).onTrue(_led.vibrantGreen()).onFalse(_led.vibrantRed());
  }

  public void updateIsValid(
      boolean shooterReady,
      boolean headingWithinTolerance, // fix later with actual heading check
      Supplier<Pose2d> robotPose) {
    isValid =
        shooterReady
            && headingWithinTolerance
            && FieldUtil.inAllianceZone(robotPose.get())
            && !convergenceFailed
            && inBound
            && FieldUtil.isHubActive();
  }

  // newton's method to find the heading to shoot at to hit the target
  public void calculateShotHeading(
      Supplier<Pose2d> robotPose,
      ChassisSpeeds robotSpeeds,
      Alliance alliance,
      boolean shooterReady,
      boolean headingWithinTolerance) {
    if (alliance == null || !FieldUtil.inAllianceZone(robotPose.get())) {
      return;
    }

    setTarget(
        alliance == Alliance.Red
            ? Constants.FieldConstants.redHub
            : Constants.FieldConstants.blueHub);

    Translation2d robotVelocity =
        new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
    Translation2d robotTranslationToTarget =
        target.getTranslation().minus(robotPose.get().getTranslation());

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
      double dT_dt = 0;

      if (robotTranslationToTarget.getNorm()
          == MathUtil.clamp(
              robotTranslationToTarget.getNorm(),
              Constants.ShooterConstants.minHubDistance,
              Constants.ShooterConstants.maxHubDistance)) {
        dT_dt =
            -robotTranslationToTarget.dot(robotVelocity)
                / (Constants.ShooterConstants.horizontolProjectileVelocity.in(MetersPerSecond)
                    * robotTranslationToTarget.getNorm());
        // pos = moving away from target
        // neg = moving closer to target
        // -(vector from robot to target dot velocity of robot) / distance to target
      }

      double error = initialGuess - tof; // guess - interpolation
      double dError_dt = 1 - dT_dt;

      if (Math.abs(error) < errorTolerance) {
        couplingDegrees =
            Degrees.of(
                Math.toDegrees(
                    Math.acos(
                        Math.abs(
                            robotTranslationToTarget.dot(robotVelocity)
                                / (robotTranslationToTarget.getNorm()
                                    * robotVelocity
                                        .getNorm()))))); // this is for checking if the robot and
        // projectile vectors overlap too much

        setVirtualTarget(target.getTranslation().minus(robotVelocity.times(initialGuess)));
        double distanceToVirtualTarget =
            virtualTarget
                .getTranslation()
                .getDistance(
                    robotPose.get().getTranslation()); // Get the distance to v target from robot

        setPreset(
            Constants.ShooterConstants.hubPresets.get(
                distanceToVirtualTarget)); // interpolate given distance

        setShotHeading(
            virtualTarget
                .getTranslation()
                .minus(robotPose.get().getTranslation())
                .getAngle()); // get shot heading given robot angle and v target angle

        newtonIterations = i + 1;
        convergenceFailed = false;

        inBound =
            Constants.ShooterConstants.minHubDistance < distanceToVirtualTarget
                && distanceToVirtualTarget < Constants.ShooterConstants.maxHubDistance;

        break;
      }

      initialGuess = initialGuess - (error / dError_dt); // update guess
    }
    if (newtonIterations == maxIterations) {
      convergenceFailed = true;
    }

    updateIsValid(shooterReady, headingWithinTolerance, robotPose);
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

  public AngularVelocity getFlywheelSpeed() {
    return _flywheelSpeed;
  }

  public AngularVelocity getRollerSpeed() {
    return _rollerSpeed;
  }

  public AngularVelocity getFloorSpeed() {
    return _floorSpeed;
  }

  public void setPreset(Matrix<N3, N1> preset) {
    _flywheelSpeed.mut_setMagnitude(preset.get(0, 0));
    _rollerSpeed.mut_setMagnitude(preset.get(1, 0));
    _floorSpeed.mut_setMagnitude(preset.get(2, 0));
  }
}
