package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;

public class IntakeFeed extends AdvancedSubsystem {
  public static final TalonFX _feedMotor = new TalonFX(Constants.IntakeConstants.feedMotorID);
  public static final VelocityVoltage feedVelocity = new VelocityVoltage(0);
  public static final StatusSignal<AngularVelocity> feedVelocityGetter = _feedMotor.getVelocity();

  public IntakeFeed() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kS = Constants.IntakeConstants.feedKs.in(Volts);
    config.Slot0.kV = Constants.IntakeConstants.feedKv.in(Volts.per(RotationsPerSecond));
    config.Slot0.kP = Constants.IntakeConstants.feedKp.in(Volts.per(RotationsPerSecond));
    config.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.feedGearRatio;

    CTREUtil.attempt(() -> _feedMotor.getConfigurator().apply(config), _feedMotor);
    FaultLogger.register(_feedMotor);

    setDefaultCommand(stop());
  }

  public void setFeedVelocity(AngularVelocity velocity) {
    _feedMotor.setControl(feedVelocity.withVelocity(velocity));
  }

  public Command stop() {
    return run(() -> _feedMotor.setControl(feedVelocity.withVelocity(0)));
  }

  public Command feed(AngularVelocity velocity) {
    return run(() -> _feedMotor.setControl(feedVelocity.withVelocity(velocity)));
  }

  @Logged
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
  public void close() {}
}
