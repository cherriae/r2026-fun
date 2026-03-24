package frc.robot.subsystems;

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
