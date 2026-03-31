// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ChangingConstants;
import frc.robot.StaticConstants;

public class SwerveTemplate extends SubsystemBase {
  
      // naming
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder absEncoder;

    private final PIDController steerPidController;

    private final boolean absEncoderReversed;
    private final double absEncoderOffsetRad;

  /** Creates a new SwerveTemplate. */
  public SwerveTemplate(int driveMotorId, int steerMotorId, int absEncoderId,
      boolean driveMotorReversed, boolean steerMotorReversed, boolean absEncoderReversed, 
      double absEncoderOffset) {

    driveMotor = new TalonFX(driveMotorId);
      // Configure the TalonFX for basic use
      TalonFXConfiguration configsDrive = new TalonFXConfiguration();
      // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and a kV of 2 on slot 0
              // got from TalonFX configuration thingy, could be entirely wrong
      configsDrive.Slot0.kP = 1;
      configsDrive.Slot0.kI = 0;
      configsDrive.Slot0.kD = 10;
      configsDrive.Slot0.kV = 2;
      configsDrive.MotorOutput.Inverted = driveMotorReversed ? 
          InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
      configsDrive.CurrentLimits.StatorCurrentLimit = ChangingConstants.CurrentLimits.kDriveStatorCurrentLimit;
      configsDrive.CurrentLimits.StatorCurrentLimitEnable = ChangingConstants.CurrentLimits.kDriveStatorLimit;
      configsDrive.CurrentLimits.SupplyCurrentLimit = ChangingConstants.CurrentLimits.kDriveSupplyCurrentLimit;
      configsDrive.CurrentLimits.SupplyCurrentLimitEnable = ChangingConstants.CurrentLimits.kDriveSupplyLimit;
      // Write these configs to the drive motor
      driveMotor.getConfigurator().apply(configsDrive);


    steerMotor = new TalonFX(steerMotorId);
      // Configure the TalonFX for basic use
      TalonFXConfiguration turningConfigs = new TalonFXConfiguration();
      // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and a kV of 2 on slot 0
              // got from TalonFX configuration thingy, could be entirely wrong
        turningConfigs.Slot0.kP = 1;
        turningConfigs.Slot0.kI = 0;
        turningConfigs.Slot0.kD = 10;
        turningConfigs.Slot0.kV = 2;
        turningConfigs.MotorOutput.Inverted = steerMotorReversed ? 
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        turningConfigs.CurrentLimits.StatorCurrentLimit = ChangingConstants.CurrentLimits.kTurningStatorCurrentLimit;
        turningConfigs.CurrentLimits.StatorCurrentLimitEnable = ChangingConstants.CurrentLimits.kTurningStatorLimit;
        turningConfigs.CurrentLimits.SupplyCurrentLimit = ChangingConstants.CurrentLimits.kTurningSupplyCurrentLimit;
        turningConfigs.CurrentLimits.SupplyCurrentLimitEnable = ChangingConstants.CurrentLimits.kTurningSupplyLimit;
      // Write these configs to the turning motor
      steerMotor.getConfigurator().apply(turningConfigs);

    this.absEncoderOffsetRad = absEncoderOffset;
    this.absEncoderReversed = absEncoderReversed;
    absEncoder = new CANcoder(absEncoderId);

    steerPidController = new PIDController(ChangingConstants.DriveConstants.kPTurning, 0, 0);
    steerPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
 }

  public double getDrivePosition() {
      var DrivePosition = driveMotor.getPosition().getValueAsDouble() * 
      StaticConstants.ModuleConstants.PhysicalConstants.kDriveEncoderRot2Meter;
    return DrivePosition;
  }

  public double getSteerPosition() {
      var SteerPosition = steerMotor.getPosition().getValueAsDouble() *
      StaticConstants.ModuleConstants.PhysicalConstants.kTurningEncoderRot2Rad;
    return SteerPosition;
  }

  public double getDriveVelocity() {
      var DriveVelocity = driveMotor.getVelocity().getValueAsDouble() * 
      StaticConstants.ModuleConstants.PhysicalConstants.kDriveEncoderRPM2MeterPerSec;
    return DriveVelocity;
  }

  public double getSteerVelocity() {
      var SteerVelocity = steerMotor.getVelocity().getValueAsDouble() *
      StaticConstants.ModuleConstants.PhysicalConstants.kTurningEncoderRPM2RadPerSec;
    return SteerVelocity;
  }

  public double getAbsoluteEncoderRad() {
    double angle = absEncoder.getPosition().getValueAsDouble();
    angle *= 2.0 * Math.PI;
    angle -= absEncoderOffsetRad;
    return angle * (absEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    driveMotor.setPosition(0);
    steerMotor.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(absEncoderOffsetRad));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(absEncoder.getPosition().getValueAsDouble()));

    driveMotor.set(desiredState.speedMetersPerSecond / 
        StaticConstants.ModuleConstants.PhysicalConstants.kPhysicalMaxSpeedMetersPerSecond);
    steerMotor.set(steerPidController.calculate(getSteerPosition(), 
        desiredState.angle.getRadians()));


    SmartDashboard.putString("Swerve[" + absEncoder.getDeviceID() + 
        "] state", desiredState.toString());
  }

  public void stop() {
    driveMotor.set(0);
    steerMotor.set(0);
  }

  public SwerveModulePosition getSwerveModulePosition() {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getSteerPosition()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // might have to put signal requester for the falcons here
  }
}
