package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/** All auton routines. */
public class Autos {
  private final AutoFactory _factory;

  private final Swerve _swerve;

  public Autos(Swerve swerve, Superstructure superstructure) {
    _swerve = swerve;

    _factory =
        new AutoFactory(
            _swerve::getPose,
            _swerve::resetPose,
            _swerve::followTrajectory,
            true,
            _swerve,
            (traj, isActive) -> {
              DogLog.log("Auto/Current Trajectory", traj.getPoses());
              DogLog.log("Auto/Current Trajectory Name", traj.name());
              DogLog.log("Auto/Current Trajectory Duration", traj.getTotalTime());
              DogLog.log("Auto/Current Trajectory Is Active", isActive);
            });

    _factory.bind("Intake", superstructure.intake());
    _factory.bind("Stop Intake", superstructure.stopIntaking());
  }

  public AutoRoutine peter(Command shootCommand) {
    AutoRoutine routine = _factory.newRoutine("peter");

    AutoTrajectory pTrajectory = routine.trajectory("peter");

    pTrajectory.atTime("Shoot").onTrue(shootCommand);

    routine.active().onTrue(sequence(pTrajectory.resetOdometry(), pTrajectory.cmd(), _swerve.brake()));

    return routine;
  }
}
