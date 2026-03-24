package frc.robot.subsystems;

public class Shooter extends AdvancedSubsystem {
    public Shooter() {}

    @Override
    public void periodic() {
        DogLog.time("Timing/Shooter/periodic()");
        super.periodic();
        DogLog.timeEnd("Timing/Shooter/periodic()");
    }

    @Override
    public void close() {}
}
