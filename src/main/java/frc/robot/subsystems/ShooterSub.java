// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */
  private TalonFX ShootingMotor = new TalonFX(Constants.ShootingMotor);
  // private RelativeEncoder ShootingEncoder;

  private CANSparkMax TurretMotor = new CANSparkMax(Constants.TurretMotor, MotorType.kBrushless);
  private RelativeEncoder TurretEncoder;

  private NetworkTable table;

  public ShooterSub() {
    // ShootingEncoder = ShootingMotor.getEncoder();
    // ShootingEncoder = ShootingMotor.En();
    // ShootingEncoder.setPositionConversionFactor(Constants.TopShooterConversionFactor);
    ShootingMotor.setInverted(true);

    TurretMotor.setInverted(true);
    TurretEncoder = TurretMotor.getEncoder();
    TurretEncoder.setPositionConversionFactor(Constants.TurretRevolutionsPerDegree);
    ResetTurretPosition(Constants.TurretStartPositionDefault);

    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TurretPosition", this.getTurretDegrees());
  }

  public void ResetTurretPosition(double degrees){
    TurretEncoder.setPosition(degrees);
  }

  public void RunShooter(double speed) {
    ShootingMotor.set(ControlMode.PercentOutput, speed);
  }

  public void StopShooter() {
    ShootingMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void setShooterPosition(double degrees) {
  }

  // public void TurnTurret(double speed) {
  //   float Kp = -0.1f;
  //   float min_command = 0.05f;

  //   NetworkTableEntry tx = table.getEntry("tx");
  //   double targetOffsetAngle_Vertical = tx.getDouble(0.0);

  //   double heading_error = -targetOffsetAngle_Vertical;
  //   double steering_adjust = 0.0f;
  //   if (targetOffsetAngle_Vertical > 1.0) {
  //     steering_adjust = Kp * heading_error - min_command;
  //   } else if (targetOffsetAngle_Vertical < 1.0) {
  //     steering_adjust = Kp * heading_error + min_command;
  //   }
  //   // TurretMotor.set(steering_adjust);
  // }

  public void StopTurret() {
    TurretMotor.set(0.0);
  }
   
public void RunTurret(double speed) {
  TurretMotor.set(speed);
  }

  public double getTurretDegrees(){
    return TurretEncoder.getPosition();
  }
}