package frc.robot.subsystems;

import dev.doglog.DogLog;
import frc.lib.AdvancedSubsystem;

public class Hopper extends AdvancedSubsystem {
  public Hopper() {}

  @Override
  public void periodic() {
    DogLog.time("Timing/Hopper/periodic()");
    super.periodic();
    DogLog.timeEnd("Timing/Hopper/periodic()");
  }

  @Override
  public void close() {}
}
