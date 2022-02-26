// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */
  private TalonFX ShootingMotor = new TalonFX (Constants.ShootingMotor);
  // private CANSparkMax TurretMotor= new CANSparkMax(Constants.TurretMotor,MotorType.kBrushless);
  private RelativeEncoder ShootingEncoder;
  private RelativeEncoder TurretEncoder;
  private NetworkTable table;
  
  public ShooterSub(){
   // ShootingEncoder = ShootingMotor.getEncoder();
  //  ShootingEncoder = ShootingMotor.En();
    // TurretEncoder = TurretMotor.getEncoder();
    // ShootingEncoder.setPositionConversionFactor(Constants.TopShooterConversionFactor);
    // TurretEncoder.setPositionConversionFactor(Constants.BottomShooterConversionFactor);
    ShootingMotor.setInverted(true);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
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
    ShootingMotor.set(ControlMode.PercentOutput, 0.0);
    
  }
  
  public void StopTurret(){
    // TurretMotor.set(0.0);
  }

  public void setShooterPosition(double degrees) {
  }

  public void TurnTurret(double speed) {
    float Kp = -0.1f;
    float min_command = 0.05f;
    
    NetworkTableEntry tx = table.getEntry("tx");
    double targetOffsetAngle_Vertical = tx.getDouble(0.0);

    double heading_error = -targetOffsetAngle_Vertical;
    double steering_adjust = 0.0f;
    if (targetOffsetAngle_Vertical > 1.0)
    {
            steering_adjust = Kp*heading_error - min_command;
    }
    else if (targetOffsetAngle_Vertical < 1.0)
    {
            steering_adjust = Kp*heading_error + min_command;
    }
    // TurretMotor.set(steering_adjust);
  }
}