package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;
import frc.lib.CTREUtil;
import frc.lib.FaultLogger;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;

public class IntakePivot extends AdvancedSubsystem {
  public static final TalonFX pivotMotor = new TalonFX(Constants.IntakeConstants.pivotMotorID, Constants.subsystemsCANBus);
  public static final PositionVoltage positionVoltage = new PositionVoltage(0);
  public static final StatusSignal<Angle> pivotAngleGetter = pivotMotor.getPosition();

  // sim 
  private final Mechanism2d _mech = new Mechanism2d(1.85, 1);
  private final MechanismRoot2d _root = _mech.getRoot("pivot", 0.5, 1);
  private final MechanismLigament2d _pivot = _root.append(new MechanismLigament2d("pivot", 0.5, 0, 3, new Color8Bit(Color.kAliceBlue)));

  private SingleJointedArmSim _actuatorSim;

  private double _lastSimTime;

  private Notifier _simNotifier;

  public IntakePivot() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;
    config.Slot0.kS = Constants.IntakeConstants.pivotKs.in(Volts);
    config.Slot0.kG = Constants.IntakeConstants.pivotKg.in(Volts);
    config.Slot0.kV = Constants.IntakeConstants.pivotKv.in(Volts.per(RadiansPerSecond));
    config.Slot0.kP = Constants.IntakeConstants.pivotKp.in(Volts.per(RadiansPerSecond));
    config.Slot0.kA = Constants.IntakeConstants.pivotKa.in(Volts.per(RadiansPerSecondPerSecond));
    config.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.pivotGearRatio;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.IntakeConstants.pivotSoftLimitForward.in(Radians);
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.IntakeConstants.pivotSoftLimitReverse.in(Radians);

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimit = 100;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // change

    config.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;

    CTREUtil.attempt(() -> pivotMotor.optimizeBusUtilization(), pivotMotor);
    CTREUtil.attempt(() -> BaseStatusSignal.setUpdateFrequencyForAll(100, pivotAngleGetter, pivotMotor.getPosition(), pivotMotor.getVelocity(), pivotMotor.getAcceleration(), pivotMotor.getStatorCurrent(), pivotMotor.getSupplyCurrent()), pivotMotor);

    CTREUtil.attempt(() -> pivotMotor.getConfigurator().apply(config), pivotMotor);
    FaultLogger.register(pivotMotor);

    if (Robot.isSimulation()) {
      // rely on sim to control the position
      pivotMotor.setPosition(0);

      // prevent setRawMotor_ from negating physics sim output
      var c = new MotorOutputConfigs();

      pivotMotor.getConfigurator().refresh(c);
      pivotMotor
          .getConfigurator()
          .apply(c.withInverted(InvertedValue.CounterClockwise_Positive));

      SmartDashboard.putData("Intake Visualizer", _mech);

      _actuatorSim =
          new SingleJointedArmSim(
              DCMotor.getKrakenX60(1),
              IntakeConstants.pivotGearRatio,
              SingleJointedArmSim.estimateMOI(
                  IntakeConstants.intakeLength.in(Meters), Units.lbsToKilograms(12)),
              IntakeConstants.intakeLength.in(Meters),
              IntakeConstants.pivotRaised.in(Radians),
              IntakeConstants.pivotLowered.in(Radians),
              true,
              IntakeConstants.pivotRaised.in(Radians));

      startSimThread();
      }

    setDefaultCommand(pivotTuck());
  }

  private void startSimThread() {
    _lastSimTime = Utils.getCurrentTimeSeconds();

    _simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - _lastSimTime;

              final double batteryVoltage = RobotController.getBatteryVoltage();

              var pivotMotorSimulationState = pivotMotor.getSimState();

              pivotMotorSimulationState.setSupplyVoltage(batteryVoltage);

              _actuatorSim.setInputVoltage(
                  pivotMotorSimulationState.getMotorVoltageMeasure().in(Volts));

              _actuatorSim.update(deltaTime);

              pivotMotorSimulationState.setRawRotorPosition(
                  Units.radiansToRotations(
                      _actuatorSim.getAngleRads() * IntakeConstants.pivotGearRatio));

              pivotMotorSimulationState.setRotorVelocity(
                  Units.radiansToRotations(
                      _actuatorSim.getVelocityRadPerSec() * IntakeConstants.pivotGearRatio));

              _lastSimTime = currentTime;
            });

    _simNotifier.setName("Intake Sim Thread");
    _simNotifier.startPeriodic(1 / Constants.simUpdateFrequency.in(Hertz));
  }

  @Logged(name = "Angle")
  public double getAngle() {
    return pivotAngleGetter.refresh().getValue().in(Radians);
  }

  public Command pivotLower() {
    return run(() -> pivotMotor.setControl(positionVoltage.withPosition(Constants.IntakeConstants.pivotLowered))).withName("Pivot Lower");
  }

  public Command pivotRaise() {
    return run(() -> pivotMotor.setControl(positionVoltage.withPosition(Constants.IntakeConstants.pivotRaised))).withName("Pivot Raise");
  }

  public Command pivotTuck() {
    return run(() -> pivotMotor.setControl(positionVoltage.withPosition(Constants.IntakeConstants.pivotTuck))).withName("Pivot Tuck");
  }

  @Override
  public void periodic() {
    DogLog.time("Timing/IntakePivot/periodic()");
    super.periodic();
    DogLog.timeEnd("Timing/IntakePivot/periodic()");
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    _pivot.setAngle(Math.toDegrees(getAngle()));
  }

  @Override
  public void close() {
    pivotMotor.close();
    _simNotifier.close();
  }

}
