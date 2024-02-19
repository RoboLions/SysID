package frc.lib.sysid;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysIdRoutinePhoenixConfig {

  /**
   * Create a new configuration for a SysId test routine.
   *
   * @param rampRate The voltage ramp rate used for quasistatic test routines. Defaults to 1 volt
   *     per second if left null.
   * @param stepVoltage The step voltage output used for dynamic test routines. Defaults to 7 volts
   *     if left null.
   * @param timeout Safety timeout for the test routine commands. Defaults to 10 seconds if left
   *     null.
   * @param logName The name for the test routine in the log. Should be unique between complete test
   *     routines (quasistatic and dynamic, forward and reverse). The current state of this test
   *     (e.g. "quasistatic-forward") will appear in CTRE Log under the "sysid-test-state-logName"
   *     signal.
   */
  public static SysIdRoutine.Config get(
      Measure<Velocity<Voltage>> rampRate,
      Measure<Voltage> stepVoltage,
      Measure<Time> timeout,
      String logName) {
    return new SysIdRoutine.Config(
        rampRate,
        stepVoltage,
        timeout,
        recordState -> {
          SignalLogger.writeString("sysid-test-state-" + logName, recordState.toString());
          return;
        });
  }
}
