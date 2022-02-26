// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSub extends SubsystemBase {
  /** Creates a new IndexSub. */
  private TalonFX FrontIndexMotor = new TalonFX(Constants.FrontIndexMotor);
  private TalonFX BackIndexMotor = new TalonFX(Constants.BackIndexMotor);
  public IndexSub() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void RunIndex(){
    FrontIndexForward();
    BackIndexForward(); 
  }
  public void RunIndexSlow(){
    FrontIndexForwardSlow();
    BackIndexForwardSlow(); 
  }
  public void ReverseIndex(){
    FrontIndexReverse();
    BackIndexReverse();
  }
  public void StopIndex(){
    StopFrontIndex();
    StopBackIndex();
  }
  public void FrontIndexForward(){
    FrontIndexMotor.set(ControlMode.PercentOutput,.95);
  }
  public void BackIndexForward(){
    BackIndexMotor.set(ControlMode.PercentOutput,-.95);
  }
  public void FrontIndexForwardSlow(){
    FrontIndexMotor.set(ControlMode.PercentOutput,.20);
  }
  public void BackIndexForwardSlow(){
    BackIndexMotor.set(ControlMode.PercentOutput,-.20);
  }

  public void FrontIndexReverse(){
    FrontIndexMotor.set(ControlMode.PercentOutput,-.75);
  }
  public void BackIndexReverse(){
    BackIndexMotor.set(ControlMode.PercentOutput,.75);
  }
  public void StopFrontIndex(){
    FrontIndexMotor.set(ControlMode.PercentOutput,0.0);
  }
  public void StopBackIndex(){
    BackIndexMotor.set(ControlMode.PercentOutput,0.0);
  }
  
  public boolean CargoLoaded(){
    //TODO: Add SensorLogic
    return true;
  }

public void indexBall() {
}
}