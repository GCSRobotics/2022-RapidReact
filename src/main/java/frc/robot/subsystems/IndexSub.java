// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSub extends SubsystemBase {
  private TalonFX FrontIndexMotor = new TalonFX(Constants.FrontIndexMotor);
  private TalonFX BackIndexMotor = new TalonFX(Constants.BackIndexMotor);

  private Rev2mDistanceSensor TopDistSensor;
  private Rev2mDistanceSensor BottomDistSensor;

  /** Creates a new IndexSub. */
  public IndexSub() {
    //Define the distance sensors using inches and a highspeed profile (slightly less accurate but speed may be important for this)
    TopDistSensor = new Rev2mDistanceSensor(Port.kMXP, Unit.kInches, RangeProfile.kHighSpeed);
    BottomDistSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kInches, RangeProfile.kHighSpeed);
    TopDistSensor.setAutomaticMode(true);
    BottomDistSensor.setAutomaticMode(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Top Range Valid", TopDistSensor.isRangeValid());
    SmartDashboard.putNumber("Top Range Inches", TopDistSensor.getRange());
    SmartDashboard.putBoolean("Bottom Range Valid", BottomDistSensor.isRangeValid());
    SmartDashboard.putNumber("Bottom Range Inches", BottomDistSensor.getRange());
    SmartDashboard.putBoolean("Cargo Indexed", this.CargoIndexed());

  }
  public void RunIndex(){
    FrontIndexForward();
    BackIndexForward(); 
  }
  public void RunIndex(double speed){
    FrontIndexForward(speed);
    BackIndexForward(speed); 
  }
  public void RunIndexSlow(){
    FrontIndexForwardSlow();
    BackIndexForwardSlow(); 
  }
  public void RunIndexSlow(double speed){
    FrontIndexForward(speed);
    BackIndexForward(speed); 
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
  public void FrontIndexForward(double speed){
    FrontIndexMotor.set(ControlMode.PercentOutput,speed);
  }
  public void BackIndexForward(double speed){
    BackIndexMotor.set(ControlMode.PercentOutput,-speed);
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
  
  public boolean CargoIndexed(){
    // Range is setup for inches
    return TopDistSensor.isRangeValid() && TopDistSensor.GetRange() <= 2.5;
  }

  public boolean CargoIncoming(){
    return BottomDistSensor.isRangeValid() && BottomDistSensor.GetRange() <= 9.0;
  }

  public void indexBall() {
  }
}