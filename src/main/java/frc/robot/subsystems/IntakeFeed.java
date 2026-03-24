package frc.robot.subsystems;

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
