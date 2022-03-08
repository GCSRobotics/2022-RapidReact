// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSub extends SubsystemBase {
  private VictorSPX ClimbMotor = new VictorSPX(Constants.ClimbMotor);
  private static final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      Constants.ClimbInChannel, Constants.ClimbOutChannel);

  /** Creates a new ClimbSub. */
  public ClimbSub() {
    ClimbMotor.setNeutralMode(NeutralMode.Brake);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("ClimbSub", this);
  }

  public void ExtendClimb() {
    ClimbMotor.set(VictorSPXControlMode.PercentOutput, 0.8);
  }

  public void RetractClimb() {
    ClimbMotor.set(VictorSPXControlMode.PercentOutput, -0.8);
  }

  public void ClimbTiltOut() {
    solenoid.set(Value.kForward);
  }

  public void ClimbTiltIn() {
    solenoid.set(Value.kReverse);
  }

  public void Stop() {
    ClimbMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }
}
