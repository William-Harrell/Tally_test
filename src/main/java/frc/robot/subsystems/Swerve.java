// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ChangingConstants;
import frc.robot.StaticConstants;

public class Swerve extends SubsystemBase {
  
  private final SwerveTemplate frontLeft = new SwerveTemplate(
    StaticConstants.MotorConstants.CanIDs.kFLDrivePort,
    StaticConstants.MotorConstants.CanIDs.kFLSteerPort,
    StaticConstants.MotorConstants.CanIDs.kFLabsEncoderPort,
    StaticConstants.MotorConstants.CanIDs.kFLDriveEncoderReversed,
    StaticConstants.MotorConstants.CanIDs.kFLSteerEncoderReversed,
    StaticConstants.MotorConstants.CanIDs.kFLabsEncoderReversed,
    ChangingConstants.DriveConstants.kFLabsEncoderOffsetRad
    );

  private final SwerveTemplate frontRight = new SwerveTemplate(
    StaticConstants.MotorConstants.CanIDs.kFRDrivePort,
    StaticConstants.MotorConstants.CanIDs.kFRSteerPort,
    StaticConstants.MotorConstants.CanIDs.kFRabsEncoderPort,
    StaticConstants.MotorConstants.CanIDs.kFRDriveEncoderReversed,
    StaticConstants.MotorConstants.CanIDs.kFRSteerEncoderReversed,
    StaticConstants.MotorConstants.CanIDs.kFRabsEncoderReversed,
    ChangingConstants.DriveConstants.kFRabsEncoderOffsetRad
    );

  private final SwerveTemplate backLeft = new SwerveTemplate(
    StaticConstants.MotorConstants.CanIDs.kBLDrivePort,
    StaticConstants.MotorConstants.CanIDs.kBLSteerPort,
    StaticConstants.MotorConstants.CanIDs.kBLabsEncoderPort,
    StaticConstants.MotorConstants.CanIDs.kBLDriveEncoderReversed,
    StaticConstants.MotorConstants.CanIDs.kBLSteerEncoderReversed,
    StaticConstants.MotorConstants.CanIDs.kBLabsEncoderReversed,
    ChangingConstants.DriveConstants.kBLabsEncoderOffsetRad
    );

  private final SwerveTemplate backRight = new SwerveTemplate(
    StaticConstants.MotorConstants.CanIDs.kBRDrivePort,
    StaticConstants.MotorConstants.CanIDs.kBRSteerPort,
    StaticConstants.MotorConstants.CanIDs.kBRabsEncoderPort,
    StaticConstants.MotorConstants.CanIDs.kBRDriveEncoderReversed,
    StaticConstants.MotorConstants.CanIDs.kBRSteerEncoderReversed,
    StaticConstants.MotorConstants.CanIDs.kBRabsEncoderReversed,
    ChangingConstants.DriveConstants.kBRabsEncoderOffsetRad
    );


  private final Pigeon2 gyro = new Pigeon2(StaticConstants.MotorConstants.CanIDs.kPigeon2Port);
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
      StaticConstants.ModuleConstants.kDriveKinematics,
      getRotation2d(), 
      new SwerveModulePosition[] {
          frontLeft.getSwerveModulePosition(), 
          frontRight.getSwerveModulePosition(), 
          backLeft.getSwerveModulePosition(), 
          backRight.getSwerveModulePosition()
      }, 
      new Pose2d(0,0, new Rotation2d(0)) 
      //TODO: change to actual starting pose when doing AUTO stuff
      );


  /** Creates a new Swerve. */
  public Swerve() {
    new Thread(() -> {
      try {
          Thread.sleep(1000);
          zeroHeading();
        } catch (Exception e) {}
    }).start();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getSwerveModulePosition(),   
      frontRight.getSwerveModulePosition(), 
      backLeft.getSwerveModulePosition(),
      backRight.getSwerveModulePosition()}, 
      pose);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getSwerveModulePosition(),   
      frontRight.getSwerveModulePosition(), 
      backLeft.getSwerveModulePosition(),
      backRight.getSwerveModulePosition()});
    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 
      StaticConstants.ModuleConstants.PhysicalConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

}
