package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.robot.Constants;

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

    setDefaultCommand(idle());
  }

  public Command idle() {
    return run(() -> leftMotor.setControl(shootVelocity.withVelocity(0))).withName("Idle");
  }

  public void setFlywheelSpeed(AngularVelocity velocity) {
    double errorRPS = flywheelGetter.getValue().in(RotationsPerSecond);

    if (Math.abs(errorRPS) > Constants.ShooterConstants.RPSTolerance) {
      inTolerance = false;
      leftMotor.setControl(dutyCycle.withOutput(1)); // 100% power if we're outside the tolerance
    } else {
      inTolerance = true;
      leftMotor.setControl(
          shootVelocity.withVelocity(velocity)); // otherwise, use the velocity control
    }
  }

  public Command shoot(AngularVelocity velocity) {
    return run(() -> setFlywheelSpeed(velocity)).withName("Shoot");
  }

  public Command spit() {
    return run(() -> leftMotor.setControl(shootVelocity.withVelocity(5))).withName("Spit");
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
  }
}
