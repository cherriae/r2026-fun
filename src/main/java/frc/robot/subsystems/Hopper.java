package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import java.util.function.Supplier;

public class Hopper extends AdvancedSubsystem {
  public static final TalonFX _feedMotor =
      new TalonFX(Constants.HopperConstants.feedMotorID, Constants.subsystemsCANBus);
  public static final TalonFX _rollerMotor =
      new TalonFX(Constants.HopperConstants.rollerMotorID, Constants.subsystemsCANBus);

  public static final VelocityVoltage feedVelocity = new VelocityVoltage(0);
  public static final VelocityVoltage rollerVelocity = new VelocityVoltage(0);

  public static final StatusSignal<AngularVelocity> feedVelocityGetter = _feedMotor.getVelocity();
  public static final StatusSignal<AngularVelocity> rollerVelocityGetter =
      _rollerMotor.getVelocity();

  private double _lastSimTime;
  private Notifier _simNotifier;
  private DCMotorSim _feedMotorSim;
  private DCMotorSim _rollerMotorSim;

  public Hopper() {
    var feedConfig = new TalonFXConfiguration();
    var rollerConfig = new TalonFXConfiguration();

    feedConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rollerConfig.Slot0.kS = Constants.HopperConstants.rollerKs.in(Volts);
    rollerConfig.Slot0.kV = Constants.HopperConstants.rollerKv.in(Volts.per(RotationsPerSecond));
    rollerConfig.Slot0.kP = Constants.HopperConstants.rollerKp.in(Volts.per(RotationsPerSecond));

    feedConfig.Slot0.kS = Constants.HopperConstants.feedKs.in(Volts);
    feedConfig.Slot0.kV = Constants.HopperConstants.feedKv.in(Volts.per(RotationsPerSecond));
    feedConfig.Slot0.kP = Constants.HopperConstants.feedKp.in(Volts.per(RotationsPerSecond));

    rollerConfig.Feedback.SensorToMechanismRatio = Constants.HopperConstants.rollerGearRatio;
    feedConfig.Feedback.SensorToMechanismRatio = Constants.HopperConstants.feedGearRatio;

    // optimize busUtil
    CTREUtil.attempt(() -> _rollerMotor.optimizeBusUtilization(), _rollerMotor);
    CTREUtil.attempt(() -> _feedMotor.optimizeBusUtilization(), _feedMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                200,
                feedVelocityGetter,
                _feedMotor.getVelocity(),
                _feedMotor.getStatorCurrent(),
                _feedMotor.getSupplyCurrent()),
        _feedMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                200,
                rollerVelocityGetter,
                _rollerMotor.getVelocity(),
                _rollerMotor.getStatorCurrent(),
                _rollerMotor.getSupplyCurrent()),
        _rollerMotor);

    // current limits
    feedConfig.CurrentLimits.SupplyCurrentLimit = 80;
    feedConfig.CurrentLimits.StatorCurrentLimit = 100;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 80;
    rollerConfig.CurrentLimits.StatorCurrentLimit = 100;

    CTREUtil.attempt(() -> _rollerMotor.getConfigurator().apply(rollerConfig), _rollerMotor);
    CTREUtil.attempt(() -> _feedMotor.getConfigurator().apply(feedConfig), _feedMotor);

    FaultLogger.register(_rollerMotor);
    FaultLogger.register(_feedMotor);

    if (frc.robot.Robot.isSimulation()) {
      _feedMotorSim =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(
                  Constants.HopperConstants.feedKv.in(Volts.per(RotationsPerSecond)), 0.01),
              Constants.MotorConstants.krakenX44);

      _rollerMotorSim =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(
                  Constants.HopperConstants.rollerKv.in(Volts.per(RotationsPerSecond)), 0.01),
              Constants.MotorConstants.krakenX44);

      startSimThread();
    }

    setDefaultCommand(stop());
  }

  private void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    _simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - _lastSimTime;
              final double batteryVoltage =
                  RobotController.getBatteryVoltage();

              var feedMotorSimulationState = _feedMotor.getSimState();
              var rollerMotorSimulationState = _rollerMotor.getSimState();

              feedMotorSimulationState.setSupplyVoltage(batteryVoltage);
              rollerMotorSimulationState.setSupplyVoltage(batteryVoltage);

              _feedMotorSim.setInputVoltage(
                  feedMotorSimulationState.getMotorVoltageMeasure().in(Volts));
              _rollerMotorSim.setInputVoltage(
                  rollerMotorSimulationState.getMotorVoltageMeasure().in(Volts));

              _feedMotorSim.update(deltaTime);
              _rollerMotorSim.update(deltaTime);

              feedMotorSimulationState.setRawRotorPosition(
                  _feedMotorSim
                      .getAngularPosition()
                      .times(Constants.HopperConstants.feedGearRatio));
              feedMotorSimulationState.setRotorVelocity(
                  _feedMotorSim
                      .getAngularVelocity()
                      .times(Constants.HopperConstants.feedGearRatio));

              rollerMotorSimulationState.setRawRotorPosition(
                  _rollerMotorSim
                      .getAngularPosition()
                      .times(Constants.HopperConstants.rollerGearRatio));
              rollerMotorSimulationState.setRotorVelocity(
                  _rollerMotorSim
                      .getAngularVelocity()
                      .times(Constants.HopperConstants.rollerGearRatio));

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("Hopper Sim Thread");
    _simNotifier.startPeriodic(
        1 / Constants.simUpdateFrequency.in(Hertz));
  }

  public void setFeedVelocity(AngularVelocity velocity) {
    _feedMotor.setControl(feedVelocity.withVelocity(velocity));
  }

  public void setRollerVelocity(AngularVelocity velocity) {
    _rollerMotor.setControl(rollerVelocity.withVelocity(velocity));
  }

  public Command feed(
      Supplier<AngularVelocity> feedVelocity, Supplier<AngularVelocity> rollerVelocity) {
    return run(() -> {
          setFeedVelocity(feedVelocity.get());
          setRollerVelocity(rollerVelocity.get());
        })
        .withName("Feed");
  }

  public Command stop() {
    return run(() -> {
          setFeedVelocity(RotationsPerSecond.zero());
          setRollerVelocity(RotationsPerSecond.zero());
        })
        .withName("Stop");
  }

  @Logged(name = "Feed Velocity")
  public AngularVelocity getFeedVelocity() {
    return feedVelocityGetter.refresh().getValue();
  }

  @Logged(name = "Roller Velocity")
  public AngularVelocity getRollerVelocity() {
    return rollerVelocityGetter.refresh().getValue();
  }

  @Override
  public void periodic() {
    DogLog.time("Timing/Hopper/periodic()");
    super.periodic();
    DogLog.timeEnd("Timing/Hopper/periodic()");
  }

  @Override
  public void close() {
    _feedMotor.close();
    _rollerMotor.close();
    if (_simNotifier != null) {
      _simNotifier.close();
    }
  }
}
