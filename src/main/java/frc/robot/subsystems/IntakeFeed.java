package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

public class IntakeFeed extends AdvancedSubsystem {
  public static final TalonFX _feedMotor = new TalonFX(Constants.IntakeConstants.feedMotorID);
  public static final VelocityVoltage feedVelocity = new VelocityVoltage(0);
  public static final StatusSignal<AngularVelocity> feedVelocityGetter = _feedMotor.getVelocity();

  private double _lastSimTime;
  private Notifier _simNotifier;
  private DCMotorSim _feedMotorSim;

  public IntakeFeed() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kS = Constants.IntakeConstants.feedKs.in(Volts);
    config.Slot0.kV = Constants.IntakeConstants.feedKv.in(Volts.per(RotationsPerSecond));
    config.Slot0.kP = Constants.IntakeConstants.feedKp.in(Volts.per(RotationsPerSecond));
    config.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.feedGearRatio;

    // optimize busUtil
    CTREUtil.attempt(() -> _feedMotor.optimizeBusUtilization(), _feedMotor);
    CTREUtil.attempt(
        () ->
            StatusSignal.setUpdateFrequencyForAll(
                200,
                feedVelocityGetter,
                _feedMotor.getVelocity(),
                _feedMotor.getSupplyCurrent(),
                _feedMotor.getStatorCurrent()),
        _feedMotor);

    // current limits
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimit = 100;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    CTREUtil.attempt(() -> _feedMotor.getConfigurator().apply(config), _feedMotor);
    FaultLogger.register(_feedMotor);

    if (frc.robot.Robot.isSimulation()) {
      _feedMotorSim =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(
                  Constants.IntakeConstants.feedKv.in(Volts.per(RotationsPerSecond)), 0.01),
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
              feedMotorSimulationState.setSupplyVoltage(batteryVoltage);

              _feedMotorSim.setInputVoltage(
                  feedMotorSimulationState.getMotorVoltageMeasure().in(Volts));

              _feedMotorSim.update(deltaTime);

              feedMotorSimulationState.setRawRotorPosition(
                  _feedMotorSim
                      .getAngularPosition()
                      .times(Constants.IntakeConstants.feedGearRatio));
              feedMotorSimulationState.setRotorVelocity(
                  _feedMotorSim
                      .getAngularVelocity()
                      .times(Constants.IntakeConstants.feedGearRatio));

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("IntakeFeed Sim Thread");
    _simNotifier.startPeriodic(
        1 / Constants.simUpdateFrequency.in(Hertz));
  }

  public void setFeedVelocity(AngularVelocity velocity) {
    _feedMotor.setControl(feedVelocity.withVelocity(velocity));
  }

  public Command stop() {
    return run(() -> _feedMotor.setControl(feedVelocity.withVelocity(0))).withName("Stop");
  }

  public Command feed(AngularVelocity velocity) {
    return run(() -> _feedMotor.setControl(feedVelocity.withVelocity(velocity))).withName("Feed");
  }

  @Logged(name = "Feed Velocity")
  public AngularVelocity getFeedVelocity() {
    return feedVelocityGetter.refresh().getValue();
  }

  @Override
  public void periodic() {
    DogLog.time("Timing/IntakeFeed/periodic()");
    super.periodic();
    DogLog.timeEnd("Timing/IntakeFeed/periodic()");
  }

  @Override
  public void close() {
    _feedMotor.close();
    if (_simNotifier != null) {
      _simNotifier.close();
    }
  }
}
