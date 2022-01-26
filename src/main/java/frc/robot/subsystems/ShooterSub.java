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
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  
  public ShooterSub(){}
  private void getEncoder() {
  }
  public void RunShooter(double speed){
    TopShootingMotor.set(.75);
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
}
