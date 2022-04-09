// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroupCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeSub.RetractIntake;
import frc.robot.commands.ShooterSub.TurnShooterDegrees;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

public class PrepForClimb extends SequentialCommandGroup {

  /** Creates a new PrepForClimb. */
  public PrepForClimb(IndexSub index, IntakeSub intake, ShooterSub shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        new ParallelCommandGroup(
            new SpitCargo(index, shooter),
            new RetractIntake(intake)).withTimeout(.7),
        new TurnShooterDegrees(shooter, 10));
  }
}
