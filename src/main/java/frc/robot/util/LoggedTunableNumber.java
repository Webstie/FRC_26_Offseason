// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A number that can be edited live from NetworkTables (e.g. AdvantageScope) for tuning, and that
 * falls back to a fixed default outside of tuning mode. Mirrors the standard AdvantageKit
 * LoggedTunableNumber: publishes under "/Tuning/{key}", logs the value, and offers {@link
 * #hasChanged}/{@link #ifChanged} so consumers only re-apply on edits.
 *
 * <p>Tuning is gated by {@link Constants#TUNING_MODE}. When it is false the NT entry is never
 * created and {@link #get()} always returns the compiled default, so competition builds carry no
 * tuning overhead.
 */
public class LoggedTunableNumber {
  private static final String TABLE_KEY = "Tuning";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new tunable number.
   *
   * @param dashboardKey Key on the dashboard, published to "/Tuning/{dashboardKey}".
   */
  public LoggedTunableNumber(String dashboardKey) {
    this.key = TABLE_KEY + "/" + dashboardKey;
  }

  /**
   * Create a new tunable number with the given default value.
   *
   * @param dashboardKey Key on the dashboard, published to "/Tuning/{dashboardKey}".
   * @param defaultValue Default value.
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value.
   */
  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (Constants.TUNING_MODE) {
        dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value.
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    }
    return Constants.TUNING_MODE ? dashboardNumber.get() : defaultValue;
  }

  /**
   * Checks whether the number has changed since the last call to this method with the given id.
   * Caller should pass a unique id (e.g. {@code hashCode()}) to identify the specific consumer.
   *
   * @param id Unique identifier for the caller.
   * @return True if the number has changed since the last time this method was called.
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  /**
   * Runs the action if any of the tunable numbers have changed.
   *
   * @param id Unique identifier for the caller.
   * @param action Callback to run when any tunable number changes; receives all values in order.
   * @param tunableNumbers All tunable numbers to check.
   */
  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    boolean anyChanged = false;
    for (LoggedTunableNumber tunableNumber : tunableNumbers) {
      if (tunableNumber.hasChanged(id)) {
        anyChanged = true;
      }
    }
    if (anyChanged) {
      double[] values = new double[tunableNumbers.length];
      for (int i = 0; i < tunableNumbers.length; i++) {
        values[i] = tunableNumbers[i].get();
      }
      action.accept(values);
    }
  }

  /** Runs the action if any of the tunable numbers have changed, with no parameters. */
  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }
}
