// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSub extends SubsystemBase {
  private CANSparkMax ClimbMotor = new CANSparkMax(Constants.ClimbMotor, MotorType.kBrushless);
  private static final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      Constants.ClimbInChannel, Constants.ClimbOutChannel);
  RelativeEncoder Encoder;

  /** Creates a new ClimbSub. */
  public ClimbSub() {
    ClimbMotor.setIdleMode(IdleMode.kBrake);
    Encoder =  ClimbMotor.getEncoder();
    Encoder.setPosition(0.0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("ClimbSub", this);
    SmartDashboard.putNumber("ClimbEncoderValue", Encoder.getPosition());
  }

  public void ExtendClimb() {
    ClimbMotor.set(1.0);
  }

  public void RetractClimb() {
    ClimbMotor.set(-1.0);
  }

  public void ClimbTiltOut() {
    solenoid.set(Value.kForward);
  }

  public void ClimbTiltIn() {
    solenoid.set(Value.kReverse);
  }

  public void Stop() {
    ClimbMotor.set(0.0);
  }

  public double GetClimbPosition(){
    return Encoder.getPosition();
  }
}
