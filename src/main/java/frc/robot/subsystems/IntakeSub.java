// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSub extends SubsystemBase {
  /** Creates a new IntakeSub. */
  private CANSparkMax IntakeMotor = new CANSparkMax(Constants.IntakeMotor, MotorType.kBrushless);
  private static final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      Constants.IntakeExtendChannel, Constants.IntakeRetractChannel);

  public IntakeSub() {
    // addChild("IntakeMotor", IntakeMotor);
    // addChild("solenoid", solenoid);
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Forward() {
    IntakeMotor.set(0.6);
  }

  public void Reverse() {
    IntakeMotor.set(-.6);

  }

  public void Stop() {
    IntakeMotor.set(0.0);
  }

  public void extendIntake() {
    // solenoid.toggle();
    solenoid.set(Value.kForward);
  }

  public void retractIntake() {
    // solenoid.toggle();
    solenoid.set(Value.kReverse);

  }
}