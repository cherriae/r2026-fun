package frc.robot.subsystems;

import dev.doglog.DogLog;
import frc.lib.AdvancedSubsystem;

public class Climb extends AdvancedSubsystem {
  public Climb() {}

  @Override
  public void periodic() {
    DogLog.time("Timing/Climb/periodic()");
    super.periodic();
    DogLog.timeEnd("Timing/Climb/periodic()");
  }

  @Override
  public void close() {}
}
