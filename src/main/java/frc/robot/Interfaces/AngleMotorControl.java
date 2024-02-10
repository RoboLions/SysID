// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Interfaces;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class AngleMotorControl {
    public static TalonFX[] motor = new TalonFX[4];
    CANcoder[] CANcoder = new CANcoder[4];

    public static final class Mod0 { 
        public static final int angleMotorID = 12;
        public static final int canCoderID = 10;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(117.334); 
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
        public static final int angleMotorID = 22;
        public static final int canCoderID = 20;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(272.285);
    }
    
    /* Back Left Module - Module 2 */
    public static final class Mod2 {
        public static final int angleMotorID = 32;
        public static final int canCoderID = 30;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(248.906);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { 
        public static final int angleMotorID = 42;
        public static final int canCoderID = 40;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(92.461);
    }

    public AngleMotorControl() {
        motor[0] = new TalonFX(Mod0.angleMotorID);
        motor[1] = new TalonFX(Mod1.angleMotorID);
        motor[2] = new TalonFX(Mod2.angleMotorID);
        motor[3] = new TalonFX(Mod3.angleMotorID);

        CANcoder[0] = new CANcoder(Mod0.canCoderID);
        CANcoder[1] = new CANcoder(Mod1.canCoderID);
        CANcoder[2] = new CANcoder(Mod2.canCoderID);
        CANcoder[3] = new CANcoder(Mod3.canCoderID);

        resetAnglePos();
    }
    public void resetAnglePos() {
        motor[0].setPosition(
        (Rotation2d.fromRotations(CANcoder[0].getAbsolutePosition().getValueAsDouble()).getDegrees()
         - Mod0.angleOffset.getDegrees())/ 360);
         motor[1].setPosition(
        (Rotation2d.fromRotations(CANcoder[1].getAbsolutePosition().getValueAsDouble()).getDegrees()
         - Mod0.angleOffset.getDegrees())/ 360);
         motor[2].setPosition(
        (Rotation2d.fromRotations(CANcoder[2].getAbsolutePosition().getValueAsDouble()).getDegrees()
         - Mod0.angleOffset.getDegrees())/ 360);
         motor[3].setPosition(
        (Rotation2d.fromRotations(CANcoder[3].getAbsolutePosition().getValueAsDouble()).getDegrees()
         - Mod0.angleOffset.getDegrees())/ 360);
    }
    public static void periodic() {
        for (TalonFX F : motor) {
            PositionDutyCycle mRequest = new PositionDutyCycle(0);
            F.setControl(mRequest);
        }
    }
}
