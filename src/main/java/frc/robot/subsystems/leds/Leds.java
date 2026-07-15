package frc.robot.subsystems.leds;

import static frc.robot.Constants.LedsConfig.*;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Status LEDs on a CTRE CANdle. Every driver action registered via {@link #addState} gets its own
 * solid color; each loop the first (highest-priority) registered action that's currently active
 * wins, falling back to the alliance color when nothing is. The CANdle self-simulates like any
 * other Phoenix 6 device, so this runs unmodified in both real and sim -- no IO split needed.
 */
public class Leds extends SubsystemBase {
  private record State(BooleanSupplier active, RGBWColor color) {}

  private final CANdle candle = new CANdle(LED_CANDLE_ID);
  private final SolidColor solidRequest = new SolidColor(0, LED_COUNT - 1);
  private final List<State> states = new ArrayList<>();

  public Leds() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;
    config.LED.BrightnessScalar = 1.0;
    candle.getConfigurator().apply(config);
  }

  /**
   * Registers a color for a driver action. Call once per action from RobotContainer, highest
   * priority first -- the first registered action whose {@code active} is true wins the strip.
   */
  public void addState(BooleanSupplier active, RGBWColor color) {
    states.add(new State(active, color));
  }

  @Override
  public void periodic() {
    for (State state : states) {
      if (state.active().getAsBoolean()) {
        candle.setControl(solidRequest.withColor(state.color()));
        return;
      }
    }
    candle.setControl(solidRequest.withColor(idleColor()));
  }

  private RGBWColor idleColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return COLOR_IDLE_NONE;
    }
    return alliance.get() == Alliance.Red ? COLOR_IDLE_RED : COLOR_IDLE_BLUE;
  }
}
