package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TunableController;

public class LightsCTRE extends SubsystemBase {
  CANdle m_candle = new CANdle(14, "chassis");
  private final int ledCount = 69;
  private Animation animation = null;
  private TunableController joystick =
      new TunableController(3).withControllerType(TunableController.TunableControllerType.LINEAR);
  private Animation m_toAnimate = null;

  /* Wrappers so we can access the CANdle from the subsystem */
  public double getVbat() {
    return m_candle.getBusVoltage();
  }

  public double get5V() {
    return m_candle.get5VRailVoltage();
  }

  public double getCurrent() {
    return m_candle.getCurrent();
  }

  public double getTemperature() {
    return m_candle.getTemperature();
  }

  public void configBrightness(double percent) {
    m_candle.configBrightnessScalar(percent, 0);
  }

  public void configLos(boolean disableWhenLos) {
    m_candle.configLOSBehavior(disableWhenLos, 0);
  }

  public void configLedType(CANdle.LEDStripType type) {
    m_candle.configLEDType(type, 0);
  }

  public void configStatusLedBehavior(boolean offWhenActive) {
    m_candle.configStatusLedState(offWhenActive, 0);
  }

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll
  }

  private AnimationTypes m_currentAnimation;

  public LightsCTRE(TunableController joy) {
    this.joystick = joy;
    changeAnimation(AnimationTypes.SetAll);
    configBrightness(100);
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = CANdle.LEDStripType.RGB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = CANdle.VBatOutputMode.On;
    m_candle.configAllSettings(configAll, 100);
  }

  public void incrementAnimation() {
    switch (m_currentAnimation) {
      case ColorFlow:
        changeAnimation(AnimationTypes.Fire);
        break;
      case Fire:
        changeAnimation(AnimationTypes.Larson);
        break;
      case Larson:
        changeAnimation(AnimationTypes.Rainbow);
        break;
      case Rainbow:
        changeAnimation(AnimationTypes.RgbFade);
        break;
      case RgbFade:
        changeAnimation(AnimationTypes.SingleFade);
        break;
      case SingleFade:
        changeAnimation(AnimationTypes.Strobe);
        break;
      case Strobe:
        changeAnimation(AnimationTypes.Twinkle);
        break;
      case Twinkle:
        changeAnimation(AnimationTypes.TwinkleOff);
        break;
      case TwinkleOff:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
      case SetAll:
        m_toAnimate = null;
        break;
    }
  }

  public void decrementAnimation() {
    switch (m_currentAnimation) {
      case ColorFlow:
        changeAnimation(AnimationTypes.TwinkleOff);
        break;
      case Fire:
        changeAnimation(AnimationTypes.ColorFlow);
        break;
      case Larson:
        changeAnimation(AnimationTypes.Fire);
        break;
      case Rainbow:
        changeAnimation(AnimationTypes.Larson);
        break;
      case RgbFade:
        changeAnimation(AnimationTypes.Rainbow);
        break;
      case SingleFade:
        changeAnimation(AnimationTypes.RgbFade);
        break;
      case Strobe:
        changeAnimation(AnimationTypes.SingleFade);
        break;
      case Twinkle:
        changeAnimation(AnimationTypes.Strobe);
        break;
      case TwinkleOff:
        changeAnimation(AnimationTypes.Twinkle);
        break;
      case SetAll:
        m_toAnimate = null;
        break;
    }
  }

  public void setColors() {
    changeAnimation(AnimationTypes.SetAll);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_toAnimate == null) {
      //      m_candle.setLEDs(0, 0, 0);
      changeAnimation(AnimationTypes.Rainbow);

    } else {
      m_candle.animate(m_toAnimate);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void changeAnimation(AnimationTypes toChange) {
    m_currentAnimation = toChange;

    switch (toChange) {
      case ColorFlow:
        m_toAnimate =
            new ColorFlowAnimation(
                128, 20, 70, 0, 0.7, ledCount, ColorFlowAnimation.Direction.Forward);
        break;
      case Fire:
        m_toAnimate = new FireAnimation(0.5, 0.7, ledCount, 0.7, 0.5);
        break;
      case Larson:
        m_toAnimate =
            new LarsonAnimation(0, 255, 46, 0, 1, ledCount, LarsonAnimation.BounceMode.Front, 3);
        break;
      case Rainbow:
        m_toAnimate = new RainbowAnimation(1, 0.1, ledCount);
        break;
      case RgbFade:
        m_toAnimate = new RgbFadeAnimation(0.7, 0.4, ledCount);
        break;
      case SingleFade:
        m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, ledCount);
        break;
      case Strobe:
        m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, ledCount);
        break;
      case Twinkle:
        m_toAnimate =
            new TwinkleAnimation(
                30, 70, 60, 0, 0.4, ledCount, TwinkleAnimation.TwinklePercent.Percent6);
        break;
      case TwinkleOff:
        m_toAnimate =
            new TwinkleOffAnimation(
                70, 90, 175, 0, 0.8, ledCount, TwinkleOffAnimation.TwinkleOffPercent.Percent100);
        break;
      case SetAll:
        m_toAnimate = null;
        break;
    }
  }
}
