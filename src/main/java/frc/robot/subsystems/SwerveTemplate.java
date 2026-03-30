// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// need to import falcons and intialize them below 
// (chaning from sparkmaxes)

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ChangingConstants;
import frc.robot.StaticConstants;
import frc.robot.StaticConstants.ModuleConstants;

public class SwerveTemplate extends SubsystemBase {
  
      // naming
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

  /** Creates a new SwerveTemplate. */
  public SwerveTemplate(int driveMotorId, int turningMotorId, boolean driveMotorReversed, 
            boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, 
            boolean absoluteEncoderReversed) {

    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANcoder(absoluteEncoderId);

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
      // Write these configs to the drive motor
      driveMotor.getConfigurator().apply(configsDrive);


    turningMotor = new TalonFX(turningMotorId);
      // Configure the TalonFX for basic use
      TalonFXConfiguration turningConfigs = new TalonFXConfiguration();
      // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and a kV of 2 on slot 0
              // got from TalonFX configuration thingy, could be entirely wrong
        turningConfigs.Slot0.kP = 1;
        turningConfigs.Slot0.kI = 0;
        turningConfigs.Slot0.kD = 10;
        turningConfigs.Slot0.kV = 2;
        turningConfigs.MotorOutput.Inverted = turningMotorReversed ? 
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
      // Write these configs to the turning motor
      turningMotor.getConfigurator().apply(turningConfigs);


    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(StaticConstants.ModuleConstants.PhysicalConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(StaticConstants.ModuleConstants.PhysicalConstants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setPositionConversionFactor(StaticConstants.ModuleConstants.PhysicalConstants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(StaticConstants.ModuleConstants.PhysicalConstants.kTurningEncoderRPM2RadPerSec);

    turningPidController = new PIDController(ChangingConstants.DriveConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
 }

  public double getDrivePosition() {
    return driveMotor.getPosition().getValueAsDouble();
  }

  public double getTurningPosition() {
    return turningMotor.getPosition().getValueAsDouble();
  }

  public double getDriveVelocity() {
    return driveMotor.getVelocity().getValueAsDouble();
  }

  public double getTurningVelocity() {
    return turningMotor.getVelocity().getValueAsDouble();
  }

    // Figure out how to use the Cancoder
  public double getAbsoluteEncoderRad() {
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }


  public void resetEncoders() {
    driveMotor.setPosition(0);
    turningMotor.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / ChangingConstants.DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // might have to put signal requester for the falcons here
  }
}
