// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class ChangingConstants {
    public static final class DriveConstants {
        
        public static final double kPTurning = 0.5; //should be good to go 

        public static final double kFLabsEncoderOffsetRad = -0.254; //maybe use the ctre generator to get vals
        public static final double kFRabsEncoderOffsetRad = -1.252;
        public static final double kBLabsEncoderOffsetRad = -1.816;
        public static final double kBRabsEncoderOffsetRad = -4.811;
        
        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * Math.PI;
        
            //Teleop speeds:
        public static final double kTeleDriveMaxSpeedMetersPerSecond = 
                                    kPhysicalMaxSpeedMetersPerSecond / 4;

        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond =
                                    kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }
    public static final class AutoConstants {
            //Auto speeds:
        public static final double kMaxSpeedMetersPerSecond = 
                                    DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = 
                                    DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 
                                    Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }
}
