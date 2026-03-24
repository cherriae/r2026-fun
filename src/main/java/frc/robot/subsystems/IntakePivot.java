package frc.robot.subsystems;

public class IntakePivot extends AdvancedSubsystem {
    public IntakePivot() {}

    @Override
    public void periodic() {
        DogLog.time("Timing/IntakePivot/periodic()");
        super.periodic();
        DogLog.timeEnd("Timing/IntakePivot/periodic()");
    }

    @Override
    public void close() {}
}
