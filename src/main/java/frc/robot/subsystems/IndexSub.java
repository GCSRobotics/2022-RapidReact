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
  private CANSparkMax UpperIndexMotor = new CANSparkMax(Constants.UpperIndexMotor, MotorType.kBrushless);
  private CANSparkMax LowerIndexMotor = new CANSparkMax(Constants.LowerIndexMotor, MotorType.kBrushless);
  public IndexSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void UpperIndexForward(){
    UpperIndexMotor.set(.75);
  }
  public void LowerIndexForward(){
    LowerIndexMotor.set(.75);
  }
  public void UpperIndexReverse(){
    UpperIndexMotor.set(-.75);
  }
  public void LowerIndexReverse(){
    LowerIndexMotor.set(-.75);
  }
  public void StopUpperIndex(){
    UpperIndexMotor.set(0.0);
  }
  public void StopLowerIndex(){
    LowerIndexMotor.set(0.0);
  }
  public void StopIndex(){
    StopUpperIndex();
    StopLowerIndex();
  }

  public boolean CargoLoaded(){
    //TODO: Add SensorLogic
    return true;
  }

public void indexBall() {
}
}