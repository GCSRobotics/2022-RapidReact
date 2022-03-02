// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeSub;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSub;

public class IntakeForward extends CommandBase {
  IntakeSub intakeSub;
  DoubleSupplier GetAxisValue;

  /** Creates a new IntakeForward. */
  public IntakeForward(IntakeSub intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSub = intake;
    addRequirements(intakeSub);
  }

  public IntakeForward(IntakeSub intake, DoubleSupplier getAxisValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSub = intake;
    GetAxisValue = getAxisValue;
    addRequirements(intakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (GetAxisValue != null) {
      intakeSub.Forward(GetAxisValue.getAsDouble());
    } else {
      intakeSub.Forward();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSub.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
