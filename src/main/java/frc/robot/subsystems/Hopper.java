package frc.robot.subsystems;

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

