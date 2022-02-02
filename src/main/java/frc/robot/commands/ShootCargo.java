// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Date;
import javax.print.attribute.standard.DateTimeAtCompleted;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.ShooterSub;

public class ShootCargo extends CommandBase {
  /** Creates a new ShootCargo. */
  IndexSub indexSub;
  ShooterSub shooterSub;
  Date initime;

  public ShootCargo(IndexSub index, ShooterSub shooter) {
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
    shooterSub.RunShooter(0.7, 0.7);
    if ((new Date()).getTime() - initime.getTime() >2000) {
      indexSub.LowerIndexForward();
   
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSub.RunShooter(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
