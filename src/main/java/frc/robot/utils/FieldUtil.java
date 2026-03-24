package frc.robot.utils;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldUtil {
    public static Optional<Alliance> alliance;
    public static double timeTolerance = 3;


    public FieldUtil() {
        System.out.println("[Field Util] Initialized");
        alliance = DriverStation.getAlliance();
    }

    public Alliance getAlliance() {
        return alliance;
    } 

    public String getGameData() {
        return DriverStation.getGameSpecificMessage();
    }

    public boolean isAuton() {
        return DriverStation.isAutonomousEnabled();
    }

    public double getMatchTime() {
        return DriverStation.getMatchTime();
    }

    public boolean isHubActive() {
        if (alliance.isEmpty() || !DriverStation.isTeleopEnabled()) {
            return false; // we don't know the alliance so there no hub or not in teleop
        }

        if (isAuton() || getGameData().isEmpty()) {
            return true; // in auton, the hub is always active
        } 

        double time = getMatchTime();
        String gameData = getGameData();

        redWonAuton = false;
        switch (gameData.charAt(0)) {
            case 'R':
                redWonAuton = true;
            case 'B': 
                redWonAuton = false;
            default:
                redWonAuton = true;
        }

        boolean shift1Active = switch (alliance) {
            case Red -> !redWonAuton;
            case Blue -> redWonAuton;
        };

          if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105 - timeTolerance) {
            // Shift 1
             return shift1Active;
        } else if (matchTime > 80 - timeTolerance) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55 - timeTolerance) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30 - timeTolerance) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }
}
