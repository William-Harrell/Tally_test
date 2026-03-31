// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class StaticConstants {
  public static class ControllerConstants { //TODO: fill in
    public static final int kDriverControllerPort = 0;
    public static final int kDriverYAxis = 0;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 0;
    public static final int kDriverFieldOrientedButtonIdx = 0;
    public static final double kDeadband = 0.05;
  }
  
  public static final class ModuleConstants { 
    public static final double kTrackWidth = Units.inchesToMeters(10.0);
        // Distance between right and left wheels                TODO: fill in
    public static final double kWheelBase = Units.inchesToMeters(10.0);
        // Distance between front and back wheels                TODO: fill in
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

      public static final class PhysicalConstants { //TODO: fill in
          //literally max speed that is physically possible
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * Math.PI;

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 5.8462; //find on SDS website
        public static final double kTurningMotorGearRatio = 1 / 18.0; //find on SDS website
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
      }
  }

  public static final class MotorConstants {

    public static final class CanIDs {
      // Random CAN IDs TODO: fill in
      public static final int kPigeon2Port = 0;

      // Front Left Module TODO: fill in
      public static final int kFLDrivePort = 0;
      public static final int kFLSteerPort = 0;
      public static final int kFLabsEncoderPort = 0;
      public static final boolean kFLDriveEncoderReversed = false;
      public static final boolean kFLSteerEncoderReversed = false;
      public static final boolean kFLabsEncoderReversed = false;

      // Front Right Module TODO: fill in
      public static final int kFRDrivePort = 0;
      public static final int kFRSteerPort = 0;
      public static final int kFRabsEncoderPort = 0;
      public static final boolean kFRDriveEncoderReversed = false;
      public static final boolean kFRSteerEncoderReversed = false;
      public static final boolean kFRabsEncoderReversed = false;

      // Back Left Module TODO: fill in
      public static final int kBLDrivePort = 0;
      public static final int kBLSteerPort = 0;
      public static final int kBLabsEncoderPort = 0;
      public static final boolean kBLDriveEncoderReversed = false;
      public static final boolean kBLSteerEncoderReversed = false;
      public static final boolean kBLabsEncoderReversed = false;

      // Back Right Module TODO: fill in
      public static final int kBRDrivePort = 0;
      public static final int kBRSteerPort = 0;
      public static final int kBRabsEncoderPort = 0;
      public static final boolean kBRDriveEncoderReversed = false;
      public static final boolean kBRSteerEncoderReversed = false;
      public static final boolean kBRabsEncoderReversed = false;


    }
  }
}