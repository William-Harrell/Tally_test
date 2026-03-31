// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class ChangingConstants {
    public static final class DriveConstants {
        
        public static final double kPTurning = 0.5; //should be good to go 

            //maybe use the ctre generator to get vals
        public static final double kFLabsEncoderOffsetRad = -0.254; 
        public static final double kFRabsEncoderOffsetRad = -1.252;
        public static final double kBLabsEncoderOffsetRad = -1.816;
        public static final double kBRabsEncoderOffsetRad = -4.811;
        
        
            //Teleop speeds:
        public static final double kTeleDriveMaxSpeedMetersPerSecond = 
            StaticConstants.ModuleConstants.PhysicalConstants.kPhysicalMaxSpeedMetersPerSecond / 4;

        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
            StaticConstants.ModuleConstants.PhysicalConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class CurrentLimits {
            // All Drive motors
        public static final double kDriveStatorCurrentLimit = 120;
        public static final boolean kDriveStatorLimit = true;
        public static final double kDriveSupplyCurrentLimit = 70;
        public static final boolean kDriveSupplyLimit = true;
            // All Turning motors
        public static final double kTurningStatorCurrentLimit = 60;
        public static final boolean kTurningStatorLimit = true;
        public static final double kTurningSupplyCurrentLimit = 50;
        public static final boolean kTurningSupplyLimit = true;
    }

    public static final class AutoConstants {
            //Auto speeds:
        public static final double kMaxSpeedMetersPerSecond = 
                StaticConstants.ModuleConstants.PhysicalConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = 
                StaticConstants.ModuleConstants.PhysicalConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }
}
