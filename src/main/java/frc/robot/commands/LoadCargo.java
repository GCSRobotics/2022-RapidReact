// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.IntakeSub;

public class LoadCargo extends CommandBase {
  IntakeSub intakeSub;
  IndexSub indexSub;
  /** Creates a new LoadCargo. */
  public LoadCargo(IntakeSub intake, IndexSub index) {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSub = intake;
    indexSub = index;
    addRequirements(intakeSub);
    addRequirements(indexSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(indexSub.CargoLoaded()){
      indexSub.StopFrontIndex();
      indexSub.StopBackIndex();
    } else {
      indexSub.FrontIndexForward();
      indexSub.BackIndexForward();
    }
    intakeSub.Forward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexSub.StopIndex();
    intakeSub.Stop();  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
