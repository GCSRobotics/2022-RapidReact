// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexSub;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSub;

public class ReverseIndex extends CommandBase {
  private final IndexSub indexSub;
  /** Creates a new ReverseIndex. */
  public ReverseIndex(IndexSub subsystem) {
    indexSub = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexSub.ReverseIndex();
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
