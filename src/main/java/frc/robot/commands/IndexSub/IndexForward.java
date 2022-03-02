// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndexSub;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSub;

public class IndexForward extends CommandBase {
  private final IndexSub indexSub;
  DoubleSupplier GetAxisValue;

  /** Creates a new RunIndex. */
  public IndexForward(IndexSub subsystem) {
    indexSub = subsystem;
  }

  public IndexForward(IndexSub subsystem, DoubleSupplier getAxisValue) {
    indexSub = subsystem;
    GetAxisValue = getAxisValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (GetAxisValue != null) {
      indexSub.RunIndex(GetAxisValue.getAsDouble());
    } else {
      indexSub.RunIndex();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexSub.StopIndex();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
