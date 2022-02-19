// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */
  private TalonFX ShootingMotor = new TalonFX (Constants.ShootingMotor);
  private CANSparkMax TurretMotor= new CANSparkMax(Constants.TurretMotor,MotorType.kBrushless);
  private RelativeEncoder ShootingEncoder;
  private RelativeEncoder TurretEncoder;
  
  public ShooterSub(){
   // ShootingEncoder = ShootingMotor.getEncoder();
    TurretEncoder = TurretMotor.getEncoder();
    ShootingEncoder.setPositionConversionFactor(Constants.TopShooterConversionFactor);
    TurretEncoder.setPositionConversionFactor(Constants.BottomShooterConversionFactor);
  }
 
  public void RunShooter(double speed){
    ShootingMotor.set(ControlMode.PercentOutput, speed);
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
  
  
  public void StopShooter(){
    ShootingMotor.set(ControlMode.PercentOutput, 0);
    
  }
  
  public void StopTurret(){
    TurretMotor.set(0.0);
  }

public void setShooterPosition(double degrees) {
}

public void TurnTurret(double speed) {
  TurretMotor.set(speed);
}
}