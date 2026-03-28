package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

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

  public AutoRoutine peter(Supplier<Command> shootCommand) {
    AutoRoutine routine = _factory.newRoutine("peter");

    AutoTrajectory pTrajectory = routine.trajectory("peter");

    pTrajectory.atTime("Shoot").onTrue(shootCommand.get().withTimeout(2));

    routine
        .active()
        .onTrue(
            sequence(
                pTrajectory.resetOdometry(),
                pTrajectory.cmd(),
                new WaitCommand(3),
                _swerve.brake()));

    return routine;
  }

  public AutoRoutine elvis(Supplier<Command> shootCommand, Supplier<Command> stopShootingCommand) {
    AutoRoutine routine = _factory.newRoutine("elvis");

    AutoTrajectory eTrajectory = routine.trajectory("elvis");

    eTrajectory.atTime("Shoot").onTrue(shootCommand.get().withTimeout(2));
    eTrajectory.atTime("Stop Shooting").onTrue(stopShootingCommand.get());

    routine
        .active()
        .onTrue(
            sequence(
                eTrajectory.resetOdometry(),
                eTrajectory.cmd(),
                new WaitCommand(3),
                _swerve.brake()));

    return routine;
  }

  public AutoRoutine peterAndJerry(
      Supplier<Command> shootCommand, Supplier<Command> stopShootingCommand) {
    AutoRoutine routine = _factory.newRoutine("peterAndJerry");

    AutoTrajectory pTrajectory = routine.trajectory("peter");
    AutoTrajectory jTrajectory = routine.trajectory("jerry");

    pTrajectory.atTime("Shoot").onTrue(shootCommand.get().withTimeout(2));

    jTrajectory.atTime("Shoot").onTrue(shootCommand.get().withTimeout(3));
    jTrajectory.atTime("Stop Shooting").onTrue(stopShootingCommand.get());

    routine
        .active()
        .onTrue(
            sequence(
                pTrajectory.resetOdometry(),
                pTrajectory.cmd(),
                new WaitCommand(3),
                jTrajectory.resetOdometry(),
                jTrajectory.cmd(),
                new WaitCommand(3),
                _swerve.brake()));

    return routine;
  }

  public AutoRoutine elvisAndRyan(
      Supplier<Command> shootCommand, Supplier<Command> stopShootingCommand) {
    AutoRoutine routine = _factory.newRoutine("elvisAndRyan");

    AutoTrajectory eTraj = routine.trajectory("elvis");
    AutoTrajectory rTraj = routine.trajectory("ryan");

    eTraj.atTime("Shoot").onTrue(shootCommand.get().withTimeout(3));

    eTraj.atTime("Stop Shooting").onTrue(stopShootingCommand.get());

    rTraj.atTime("Shoot").onTrue(shootCommand.get());

    rTraj.atTime("Stop Shooting").onTrue(stopShootingCommand.get());

    routine
        .active()
        .onTrue(
            sequence(
                eTraj.resetOdometry(),
                eTraj.cmd(),
                new WaitCommand(3),
                rTraj.resetOdometry(),
                rTraj.cmd(),
                new WaitCommand(3),
                _swerve.brake()));

    return routine;
  }
}
