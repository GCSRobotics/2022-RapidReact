// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */
  private CANSparkMax TopShootingMotor = new CANSparkMax(Constants.TopShootingMotor, MotorType.kBrushless);
  private CANSparkMax BottomShootingMotor = new CANSparkMax(Constants.BottomShootingMotor,MotorType.kBrushless);
  private RelativeEncoder TopEncoder;
  private RelativeEncoder BottomEncoder;
  
  public ShooterSub(){
    TopEncoder = TopShootingMotor.getEncoder();
    BottomEncoder = BottomShootingMotor.getEncoder();
    TopEncoder.setPositionConversionFactor(Constants.TopShooterConversionFactor);
    BottomEncoder.setPositionConversionFactor(Constants.BottomShooterConversionFactor);
  }
  private void getEncoder() {
  }
  public void RunShooter(int lowerspeed){
    BottomShootingMotor.set(lowerspeed);
  }
  
  public void RunShooter(double upperspeed,double lowerspeed){
    TopShootingMotor.set(upperspeed);
    BottomShootingMotor.set(lowerspeed);
  }
  public double GetshooterSpeed(){
    GetEncodervalue(0.75);
    return 0.6;
  }

  private void GetEncodervalue(double d) {
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void StopTopShooter(){
    TopShootingMotor.set(0.0);
  }
  
  public void StopBottomShooter(){
    BottomShootingMotor.set(0.0);
  }
}