package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.robot.Constants;
import java.util.function.Supplier;

public class Shooter extends AdvancedSubsystem {
  public static final TalonFX leftMotor =
      new TalonFX(Constants.ShooterConstants.shooterLeftFlywheelID, Constants.subsystemsCANBus);
  // follower, inverted
  public static final TalonFX rightMotor =
      new TalonFX(Constants.ShooterConstants.shooterRightFlywheelID, Constants.subsystemsCANBus);

  public static final VelocityVoltage shootVelocity = new VelocityVoltage(0);
  public static final DutyCycleOut dutyCycle = new DutyCycleOut(0);
  public static final VoltageOut voltageOut = new VoltageOut(0);

  public static final StatusSignal<AngularVelocity> flywheelGetter = leftMotor.getVelocity();

  public static boolean inTolerance = false;

  // public static final StatusSignal<AngularVelocity> rightVelocitySignal =
  // rightMotor.getVelocity();

  private double _lastSimTime;
  private Notifier _simNotifier;
  private FlywheelSim _flywheelSim;

  public Shooter() {
    var leftConfig = new TalonFXConfiguration();
    var rightConfig = new TalonFXConfiguration();

    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    leftConfig.Slot0.kS = Constants.ShooterConstants.flywheelkS.in(Volts);
    leftConfig.Slot0.kV = Constants.ShooterConstants.flywheelkV.in(Volts.per(RotationsPerSecond));
    leftConfig.Slot0.kP = Constants.ShooterConstants.flywheelkP.in(Volts.per(RotationsPerSecond));

    leftConfig.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.shooterGearRatio;

    // current limits
    leftConfig.CurrentLimits.SupplyCurrentLimit = 60;
    rightConfig.CurrentLimits.SupplyCurrentLimit = 60;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    leftConfig.CurrentLimits.StatorCurrentLimit = 100;
    rightConfig.CurrentLimits.StatorCurrentLimit = 100;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // optimize busUtil
    CTREUtil.attempt(() -> leftMotor.optimizeBusUtilization(), leftMotor);
    CTREUtil.attempt(() -> rightMotor.optimizeBusUtilization(), rightMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                200,
                flywheelGetter,
                leftMotor.getVelocity(),
                leftMotor.getPosition(),
                leftMotor.getSupplyCurrent(),
                leftMotor.getStatorCurrent()),
        leftMotor);

    CTREUtil.attempt(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                rightMotor.getVelocity(),
                rightMotor.getSupplyCurrent(),
                rightMotor.getStatorCurrent()),
        rightMotor);

    rightConfig.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.shooterGearRatio;

    CTREUtil.attempt(() -> leftMotor.getConfigurator().apply(leftConfig), leftMotor);
    CTREUtil.attempt(() -> rightMotor.getConfigurator().apply(rightConfig), rightMotor);

    rightMotor.setControl(
        new Follower(
            Constants.ShooterConstants.shooterRightFlywheelID, MotorAlignmentValue.Opposed));

    if (frc.robot.Robot.isSimulation()) {
      _flywheelSim =
          new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                  Constants.MotorConstants.krakenX44,
                  0.01, // MOI
                  Constants.ShooterConstants.shooterGearRatio),
              Constants.MotorConstants.krakenX44);

      startSimThread();
    }

    setDefaultCommand(idle());
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

              var leftSim = leftMotor.getSimState();
              var rightSim = rightMotor.getSimState();

              leftSim.setSupplyVoltage(batteryVoltage);
              rightSim.setSupplyVoltage(batteryVoltage);

              // Flywheel takes main motor's voltage
              _flywheelSim.setInputVoltage(leftSim.getMotorVoltageMeasure().in(Volts));
              _flywheelSim.update(deltaTime);

              double mechanismRPS = _flywheelSim.getAngularVelocityRPM() / 60.0;

              leftSim.setRotorVelocity(mechanismRPS * Constants.ShooterConstants.shooterGearRatio);
              rightSim.setRotorVelocity(
                  mechanismRPS * Constants.ShooterConstants.shooterGearRatio); // follower opposed

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("Shooter Sim Thread");
    _simNotifier.startPeriodic(
        1 / Constants.simUpdateFrequency.in(Hertz));
  }

  public Command idle() {
    return run(() ->
            leftMotor.setControl(shootVelocity.withVelocity(Constants.ShooterConstants.idleRPS)))
        .withName("Idle");
  }

  public void setFlywheelSpeed(AngularVelocity velocity) {
    double current = flywheelGetter.getValue().in(RotationsPerSecond);
    double target = velocity.in(RotationsPerSecond);
    double error = target - current;

    if (Math.abs(error) < Constants.ShooterConstants.RPSTolerance.in(RotationsPerSecond)) {
      inTolerance = true;
    } else {
      inTolerance = false;
    }
    if (Math.abs(error) > Constants.ShooterConstants.BangBangRPSTolerane.in(RotationsPerSecond)) {
      leftMotor.setControl(dutyCycle.withOutput(1)); // 100% power if we're outside the tolerance
    } else {
      leftMotor.setControl(
          shootVelocity.withVelocity(velocity)); // otherwise, use the velocity control
    }
  }

  public Command shoot(Supplier<AngularVelocity> velocitySupplier) {
    return run(() -> setFlywheelSpeed(velocitySupplier.get())).withName("Shoot");
  }

  public Command spit() {
    return run(() ->
            leftMotor.setControl(shootVelocity.withVelocity(Constants.ShooterConstants.spitRPS)))
        .withName("Spit");
  }

  @Logged(name = "Flywheel Speed")
  public AngularVelocity getFlywheelSpeed() {
    return flywheelGetter.refresh().getValue();
  }

  @Override
  public void periodic() {
    DogLog.time("Timing/Shooter/periodic()");
    super.periodic();
    DogLog.timeEnd("Timing/Shooter/periodic()");
  }

  @Override
  public void close() {
    leftMotor.close();
    rightMotor.close();
    if (_simNotifier != null) {
      _simNotifier.close();
    }
  }
}
