// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.sysid.SysIdRoutinePhoenixConfig;

public class SysIdSubsystem extends SubsystemBase {

  private final TalonFX m_motor0 = new TalonFX(0, "rio");
  private final TalonFX m_motor1 = new TalonFX(1, "rio");
  private final TalonFX m_motor2 = new TalonFX(2, "rio");
  private final TalonFX m_motor3 = new TalonFX(3, "rio");

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          SysIdRoutinePhoenixConfig.get(
              Volts.per(Second).of(1.0), Volts.of(7.0), Seconds.of(10.0), this.getName()),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                m_motor0.setControl(new VoltageOut(volts.in(Volts)));
              },
              // Phoenix 6 will log all the necessary status signals by default.
              // No need to define a log routine
              log -> {},
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  /** Creates a new Drive subsystem. */
  public SysIdSubsystem() {
    m_motor1.setControl(new Follower(m_motor0.getDeviceID(), false));
    m_motor2.setControl(new Follower(m_motor0.getDeviceID(), false));
    m_motor3.setControl(new Follower(m_motor0.getDeviceID(), false));
    // Setting the update frequency of the signals will determine the log file resolution
    configureStatusSignals(m_motor0, 1000.0);
    configureStatusSignals(m_motor1, 1000.0);
    configureStatusSignals(m_motor2, 1000.0);
    configureStatusSignals(m_motor3, 1000.0);
  }

  /**
   * Configure the Status Signals required for SysId of the motor to have the given logging rate.
   *
   * @param motor The {@link TalonFX} motor for which to configure the status signals
   * @param frequencyHz The desired logging frequency of the signals in Hz
   */
  private void configureStatusSignals(TalonFX motor, double frequencyHz) {
    motor.getMotorVoltage().setUpdateFrequency(frequencyHz);
    motor.getPosition().setUpdateFrequency(frequencyHz);
    motor.getVelocity().setUpdateFrequency(frequencyHz);
    motor.optimizeBusUtilization();
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /** Returns the full SysId Command Sequence */
  public Command fullSysIdCommand() {
    return runOnce(
            () -> {
              SignalLogger.start();
            })
        .andThen(this.sysIdQuasistatic(Direction.kForward))
        .andThen(new WaitCommand(3))
        .andThen(this.sysIdQuasistatic(Direction.kReverse))
        .andThen(new WaitCommand(3))
        .andThen(this.sysIdDynamic(Direction.kForward))
        .andThen(new WaitCommand(3))
        .andThen(this.sysIdDynamic(Direction.kReverse))
        .andThen(
            () -> {
              SignalLogger.stop();
            });
  }
}
