// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroupCommands;

import java.util.Date;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.ShooterSub;

public class ShootCargoAuton extends CommandBase {
  /** Creates a new ShootCargo. */
  IndexSub indexSub;
  ShooterSub shooterSub;
  Date initime;

  public ShootCargoAuton(IndexSub index, ShooterSub shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    indexSub = index;
    shooterSub = shooter;
    addRequirements(indexSub);
    addRequirements(shooterSub);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initime = new Date();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSub.RunShooter(0.40);
    // if time is run than 2 seconds to turn on the index
    long timePassedMil = (new Date()).getTime() - initime.getTime();
    if (timePassedMil > 500) {
      if (indexSub.CargoIndexed() &&
          timePassedMil < 1000) {
        indexSub.StopIndex();
      } else {
        indexSub.RunIndex(0.8);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSub.StopShooter();
    indexSub.StopIndex();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
