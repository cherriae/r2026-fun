package frc.robot.subsystems;

import dev.doglog.DogLog;
import frc.lib.AdvancedSubsystem;

public class IntakeFeed extends AdvancedSubsystem {
  public IntakeFeed() {}

  @Override
  public void periodic() {
    DogLog.time("Timing/IntakeFeed/periodic()");
    super.periodic();
    DogLog.timeEnd("Timing/IntakeFeed/periodic()");
  }

  @Override
  public void close() {}
}
