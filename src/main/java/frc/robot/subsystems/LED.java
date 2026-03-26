package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AdvancedSubsystem;

public class LED extends AdvancedSubsystem {
  private static AddressableLED led;
  private static AddressableLEDBuffer buffer;

  public LED(int port, int bufferLength) {
    led = new AddressableLED(port);
    buffer = new AddressableLEDBuffer(bufferLength);

    led.setLength(bufferLength);

    led.start();

    setDefaultCommand(vibrantBlue());
  }

  // idle
  public Command vibrantBlue() {
    return run(
        () -> {
          LEDPattern pattern =
              LEDPattern.gradient(GradientType.kContinuous, Color.kBlue, Color.kCyan);
          pattern.applyTo(buffer);
        });
  }

  // isValid = false
  public Command vibrantRed() {
    return run(
        () -> {
          LEDPattern pattern =
              LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kOrange);
          pattern.applyTo(buffer);
        });
  }

  // isValid = true
  public Command vibrantGreen() {
    return run(
        () -> {
          LEDPattern pattern =
              LEDPattern.gradient(GradientType.kContinuous, Color.kGreen, Color.kLime);
          pattern.blink(Seconds.of(0.5));
          pattern.applyTo(buffer);
        });
  }

  // intaking
  public Command vibrantYellow() {
    return run(
        () -> {
          LEDPattern pattern =
              LEDPattern.gradient(GradientType.kContinuous, Color.kYellow, Color.kGold);
          pattern.applyTo(buffer);
        });
  }

  // climbing
  public Command vibrantPurple() {
    return run(
        () -> {
          LEDPattern pattern =
              LEDPattern.gradient(GradientType.kContinuous, Color.kPurple, Color.kMagenta);
          pattern.applyTo(buffer);
        });
  }

  public void setDefaultColor() {
    SmartDashboard.putData("Vibrant Red", run( () -> setDefaultCommand(vibrantRed())));
    SmartDashboard.putData("Vibrant Blue", run( () -> setDefaultCommand(vibrantBlue())));
    SmartDashboard.putData("Vibrant Green", run( () -> setDefaultCommand(vibrantGreen())));
    SmartDashboard.putData("Vibrant Yellow", run( () -> setDefaultCommand(vibrantYellow())));
    SmartDashboard.putData("Vibrant Purple", run( () -> setDefaultCommand(vibrantPurple())));
  }

  @Override
  public void periodic() {
    led.setData(buffer);
  }

  @Override
  public void close() {
    led.close();
  }
}
