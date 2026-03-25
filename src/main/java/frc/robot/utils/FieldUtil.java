package frc.robot.utils;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import java.util.Optional;

public class FieldUtil {
  public static Optional<Alliance> alliance = Optional.empty();
  public static double timeTolerance = 3;

  private static double prevSwitchTime = 110;

  public FieldUtil() {
    System.out.println("[Field Util] Initialized");
    alliance = DriverStation.getAlliance();
  }

  public Optional<Alliance> getAlliance() {
    return alliance;
  }

  public String getGameData() {
    return DriverStation.getGameSpecificMessage();
  }

  public static boolean isAuton() {
    return DriverStation.isAutonomousEnabled();
  }

  public double getMatchTime() {
    return DriverStation.getMatchTime();
  }

  public boolean isHubActive() {
    if (isAuton() || getMatchTime() <= 30) { // endgame
      return true;
    }

    String data = DriverStation.getGameSpecificMessage();
    boolean active = false;

    switch (alliance.get()) {
      case Blue:
        active = data.charAt(0) == 'B';
        break;

      case Red:
        active = data.charAt(0) == 'R';
        break;
    }

    if (prevSwitchTime - 25 >= DriverStation.getMatchTime()) {
      active = !active;
      prevSwitchTime = DriverStation.getMatchTime();
    }

    return active;
  }

  /** Whether the supplied robot pose is in the bump zone(s). */
  public static boolean inBumpZone(Pose2d robotPose) {
    if (FieldConstants.blueBumpZone.contains(robotPose.getTranslation())
        || FieldConstants.redBumpZone.contains(robotPose.getTranslation())) {
      return true;
    }

    return false;
  }

  public static boolean inAllianceZone(Pose2d robotPose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      return FieldConstants.blueAllianceZone.contains(robotPose.getTranslation());
    } else {
      return FieldConstants.redAllianceZone.contains(robotPose.getTranslation());
    }
  }

  public static boolean inNeutralZone(Pose2d robotPose) {
    return FieldConstants.neutralZone.contains(robotPose.getTranslation());
  }

  public void log(Pose2d robotPose) {
    DogLog.log("FieldUtil/Alliance", alliance.get().toString());
    DogLog.log("FieldUtil/Is Hub Active", isHubActive());
    DogLog.log("FieldUtil/In Bump Zone", inBumpZone(robotPose));
    DogLog.log("FieldUtil/In Alliance Zone", inAllianceZone(robotPose));
    DogLog.log("FieldUtil/In Neutral Zone", inNeutralZone(robotPose));
  }
}
