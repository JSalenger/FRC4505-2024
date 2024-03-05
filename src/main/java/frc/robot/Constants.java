// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivebaseConstants {
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAngularSpeed = 2 * 2 * Math.PI;
    public static final double kTeleMaxDriveAccelerationMetersPerSecond = 3;
    public static final double kTeleMaxAngularAccelerationMetersPerSecond = 3;

    public static final double kTeleMaxDriveSpeed = kMaxSpeedMetersPerSecond * 0.75;  // /2
    public static final double kTeleMaxAngularSpeed = kMaxAngularSpeed / 4;

    public static final double kTrackWidth = Units.inchesToMeters(29); // diff btw left and right
    public static final double kWheelBase = Units.inchesToMeters(29);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth/ 2), // front left
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // front right
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // back right
      new Translation2d(-kWheelBase /2, kTrackWidth / 2) // back left
    );
  }

  public static class SwerveModuleConstants {
    public static final double kWheelDiamteterMeters = Units.inchesToMeters(4); // TODO: CHECK
    public static final double kDriveMotorGearRatio = 16/50;  // try 14/50 once the auto works again//1 / 5.8462;
    public static final double kTurningMotorGearRatio = 1 / 18.0;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiamteterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.75d;  // 0.5d
  }

  // https://github.com/SeanSun6814/FRC0ToAutonomous/blob/master/%236%20Swerve%20Drive%20Auto/src/main/java/frc/robot/Constants.java
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadband = 0.05;
}
  public static final class AutoConstants {
    public static final double kPXController = 3;  // all kP previously 1
    public static final double kPYController = 3;
    public static final double kPAngleController = 15;  // 5
    public static final double kMaxSpeedMetersPerSecond = 4;  // 3
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;  // 3

    public static final TrapezoidProfile.Constraints kAngleControllerConstraints = 
      new TrapezoidProfile.Constraints(DrivebaseConstants.kMaxAngularSpeed, DrivebaseConstants.kTeleMaxAngularAccelerationMetersPerSecond);
  }
}
