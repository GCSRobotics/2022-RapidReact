// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveSub;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {
  // Define Motors
  private CANSparkMax leftFrontMotor = new CANSparkMax(Constants.LeftFrontDriveMotor, MotorType.kBrushless);
  private CANSparkMax leftRearMotor = new CANSparkMax(Constants.LeftRearDriveMotor, MotorType.kBrushless);
  private CANSparkMax rightFrontMotor = new CANSparkMax(Constants.RightFrontDriveMotor, MotorType.kBrushless);
  private CANSparkMax rightRearMotor = new CANSparkMax(Constants.RightRearDriveMotor, MotorType.kBrushless);

  MotorControllerGroup leftMotorGroup = new MotorControllerGroup(leftFrontMotor, leftRearMotor);
  MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rightFrontMotor, rightRearMotor);

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private final DifferentialDrive robotDrive;

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS1);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    rightMotorGroup.setInverted(true);

    leftEncoder = leftFrontMotor.getEncoder();
    rightEncoder = rightFrontMotor.getEncoder();
    leftEncoder.setPositionConversionFactor(Constants.InchesPerMotorRotation);
    rightEncoder.setPositionConversionFactor(Constants.InchesPerMotorRotation);

    robotDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

// ********************************************
  // DRIVE - Methods to Drive robot
  // ********************************************
  public void arcadeDrive(double speedAxis, double rotationAxis) {
    robotDrive.arcadeDrive(speedAxis, rotationAxis, true);
  }

  public void arcadeDrive(double speedAxis, double rotationAxis, boolean squared) {
    robotDrive.arcadeDrive(speedAxis, rotationAxis, squared);
  }

  public void setMaxDriveSpeed(double maxSpeed){
    robotDrive.setMaxOutput(maxSpeed);
  }

  public void stop() {
    robotDrive.arcadeDrive(0, 0);
  }
  // ********************************************
  // ENCODER - Methods to get Encoder Readings
  // ********************************************
  public void resetEncoders() {
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
  }

  public double getLeftDistanceInch() {
    return leftEncoder.getPosition();
  }

  public double getRightDistanceInch() {
    return rightEncoder.getPosition();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  // ********************************************
  // Gyro - Methods to get Gyro Readings
  // ********************************************
  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public double getGyroRate() {
    return gyro.getRate();
  }
}
