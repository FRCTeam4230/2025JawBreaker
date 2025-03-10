package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsCTRE extends SubsystemBase {
  // CANdle candle = new CANdle(0);
  private final int ledCount = 8;
  private Animation animation = null;

  //    @Override
  //    public void periodic() {
  //        candle.setLEDs(255, 255, 255);
  //    }
}

// examples of things capable
/*
       {
           case ColorFlow:
               animation = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
               break;
           case Fire:
               animation = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
               break;
           case Larson:
               animation = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
               break;
           case Rainbow:
               animation = new RainbowAnimation(1, 0.1, LedCount);
               break;
           case RgbFade:
               animation = new RgbFadeAnimation(0.7, 0.4, LedCount);
               break;
           case SingleFade:
               animation = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
               break;
           case Strobe:
               animation = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
               break;
           case Twinkle:
               animation = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
               break;
           case TwinkleOff:
               animation = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
               break;
           case SetAll:
               animation = null;
               break;
       }
*/
