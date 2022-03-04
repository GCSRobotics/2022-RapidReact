// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IndexSub;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSub;

public class IndexAutomatic extends CommandBase {
  private final IndexSub indexSub;
  boolean loading = false;

  /** Creates a new RunIndex. */
  public IndexAutomatic(IndexSub subsystem) {
    indexSub = subsystem;
    addRequirements(indexSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (indexSub.CargoIncoming()) {
      loading = true;
      indexSub.RunIndex(0.6);
    } else if (indexSub.CargoIndexed()) {
      loading = false;
      indexSub.StopIndex();
    } else {
      if (!loading) {
        indexSub.StopIndex();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
