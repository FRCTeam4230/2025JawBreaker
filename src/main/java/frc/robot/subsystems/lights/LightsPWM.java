package frc.robot.subsystems.lights;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsPWM extends SubsystemBase {

  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;

  Color marshallGold = new Color(75, 249, 0); // g, r, b

  LEDPattern base =
      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, marshallGold, Color.kBlack);

  LEDPattern breathePattern = base.breathe(Seconds.of(1));
  LEDPattern scrollPattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(50));

  LEDPattern color = scrollPattern;

  public LightsPWM() {
    led = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(315); // 57 real
    led.setLength(ledBuffer.getLength());
    color.applyTo(ledBuffer);

    led.setData(ledBuffer);
    led.start();
  }

  public Command runPattern(LEDPattern pattern) {
    //        return run(() -> pattern.applyTo(ledBuffer));
    color = pattern;
    return Commands.runOnce(() -> pattern.applyTo(ledBuffer))
        .andThen(Commands.runOnce(() -> led.setData(ledBuffer)));
  }

  @Override
  public void periodic() {
    color.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  public Command scrollPattern() {
    //    LEDPattern base =
    //        LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlue,
    // Color.kYellow);

    return runPattern(scrollPattern);
  }
}
