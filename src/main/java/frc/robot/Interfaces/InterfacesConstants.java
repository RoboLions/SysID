// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Interfaces;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class InterfacesConstants {
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.0);
    public static final double WHEEL_BASE = Units.inchesToMeters(22.0); 
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.9);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

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
}
