// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.interfaces;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class AngleMotorControl {

  public static class ModConfig {
    public final int m_angleMotorID;
    public final int m_canCoderID;
    public final Rotation2d m_canCoderOffset;

    public ModConfig(int angleMotorID, int canCoderID, Rotation2d canCoderOffset) {
      m_angleMotorID = angleMotorID;
      m_canCoderID = canCoderID;
      m_canCoderOffset = canCoderOffset;
    }
  }

  public static final ModConfig[] m_modConfigs =
      new ModConfig[] {
        new ModConfig(12, 10, Rotation2d.fromDegrees(117.334)),
        new ModConfig(22, 20, Rotation2d.fromDegrees(272.285)),
        new ModConfig(32, 30, Rotation2d.fromDegrees(248.906)),
        new ModConfig(42, 40, Rotation2d.fromDegrees(92.461)),
      };

  public static final TalonFX[] m_angleMotors =
      new TalonFX[] {
        new TalonFX(m_modConfigs[0].m_angleMotorID),
        new TalonFX(m_modConfigs[1].m_angleMotorID),
        new TalonFX(m_modConfigs[2].m_angleMotorID),
        new TalonFX(m_modConfigs[3].m_angleMotorID),
      };

  public static final CANcoder[] m_canCoders =
      new CANcoder[] {
        new CANcoder(m_modConfigs[0].m_canCoderID),
        new CANcoder(m_modConfigs[1].m_canCoderID),
        new CANcoder(m_modConfigs[2].m_canCoderID),
        new CANcoder(m_modConfigs[3].m_canCoderID),
      };

  public AngleMotorControl() {
    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();

    angleMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
    angleMotorConfig.Feedback.SensorToMechanismRatio = 150.0 / 7.0;
    angleMotorConfig.MotionMagic.MotionMagicAcceleration = 1.0;
    angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    angleMotorConfig.MotionMagic.MotionMagicExpo_kA = 0.17079;
    angleMotorConfig.MotionMagic.MotionMagicExpo_kV = 2.314285;
    angleMotorConfig.MotionMagic.MotionMagicJerk = 1.0;
    angleMotorConfig.Slot0.kS = 0.16992;
    angleMotorConfig.Slot0.kV = 2.314285;
    angleMotorConfig.Slot0.kA = 0.17079;
    angleMotorConfig.Slot0.kP = 107.142857;
    angleMotorConfig.Slot0.kI = 1.0;
    angleMotorConfig.Slot0.kD = 1.0;
    angleMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    for (int modIdx = 0; modIdx < m_modConfigs.length; modIdx++) {
      canCoderConfig.MagnetSensor.MagnetOffset =
          m_modConfigs[modIdx].m_canCoderOffset.getRotations();
      m_angleMotors[modIdx].getConfigurator().apply(angleMotorConfig);
      m_canCoders[modIdx].getConfigurator().apply(canCoderConfig);
      m_angleMotors[modIdx].setPosition(m_canCoders[modIdx].getPosition().refresh().getValue());
    }
  }

  public static void periodic() {
    for (TalonFX angleMotor : m_angleMotors) {
      angleMotor.setControl(new MotionMagicExpoVoltage(0));
    }
  }
}
