// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSub extends SubsystemBase {
  /** Creates a new IndexSub. */
  private CANSparkMax FrontIndexMotor = new CANSparkMax(Constants.FrontIndexMotor, MotorType.kBrushless);
  private CANSparkMax BackIndexMotor = new CANSparkMax(Constants.BackIndexMotor, MotorType.kBrushless);
  public IndexSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void FrontIndexForward(){
    FrontIndexMotor.set(.75);
  }
  public void BackIndexForward(){
    BackIndexMotor.set(.75);
  }
  public void FrontIndexReverse(){
    FrontIndexMotor.set(-.75);
  }
  public void BackIndexReverse(){
    BackIndexMotor.set(-.75);
  }
  public void StopFrontIndex(){
    FrontIndexMotor.set(0.0);
  }
  public void StopBackIndex(){
    BackIndexMotor.set(0.0);
  }
  public void StopIndex(){
    StopFrontIndex();
    StopBackIndex();
  }

  public boolean CargoLoaded(){
    //TODO: Add SensorLogic
    return true;
  }

public void indexBall() {
}
}