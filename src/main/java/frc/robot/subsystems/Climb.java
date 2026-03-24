package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;

public class Climb extends AdvancedSubsystem {
  public static final TalonFX _climbMotor =
      new TalonFX(Constants.ClimberConstants.ClimberMotorID, Constants.subsystemsCANBus);

  public Climb() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kS = Constants.ClimberConstants.climberKs.in(Volts);
    config.Slot0.kV = Constants.ClimberConstants.climberKv.in(Volts.per(MetersPerSecond));
    config.Slot0.kP = Constants.ClimberConstants.climberKp.in(Volts.per(MetersPerSecond));
    config.Slot0.kG = Constants.ClimberConstants.climberKg.in(Volts);
    config.Slot0.kA = Constants.ClimberConstants.climberKa.in(Volts.per(Radians));

    // need climbing constants later

    CTREUtil.attempt(() -> _climbMotor.getConfigurator().apply(config), _climbMotor);
    FaultLogger.register(_climbMotor);
  }

  @Override
  public void periodic() {
    DogLog.time("Timing/Climb/periodic()");
    super.periodic();
    DogLog.timeEnd("Timing/Climb/periodic()");
  }

  @Override
  public void close() {}
}
