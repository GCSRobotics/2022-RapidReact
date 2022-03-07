// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbSub;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSub;

public class RetractClimb extends CommandBase {
  ClimbSub climbSub;
  /** Creates a new RetractClimb. */
  public RetractClimb(ClimbSub climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    climbSub = climb;
    addRequirements(climbSub);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSub.RetractClimb();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSub.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
