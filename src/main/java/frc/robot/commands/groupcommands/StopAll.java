// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroupCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

public class StopAll extends CommandBase {
  private ShooterSub shootersub;
  private IndexSub indexsub;
  private IntakeSub intakesub;
  private DriveSubsystem drivesub;
  /** Creates a new StopAll. */
  public StopAll(ShooterSub shooter, IndexSub index, IntakeSub intake, DriveSubsystem drive) { 
    shootersub = shooter;
    indexsub = index;
    intakesub = intake;
    drivesub = drive;
   // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakesub.Stop(); 
    indexsub.StopIndex();
    shootersub.StopShooter();
    drivesub.StopAll();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
