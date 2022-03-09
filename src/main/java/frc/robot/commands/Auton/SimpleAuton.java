// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.GroupCommands.ShootCargoAuton;
import frc.robot.commands.GroupCommands.StopAll;
import frc.robot.commands.IntakeSub.ExtendIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

public class SimpleAuton extends SequentialCommandGroup {
    public SimpleAuton(DriveSubsystem drivetrain, ShooterSub shooter, IndexSub index, IntakeSub intake) {
        double driveSpeed = 0.40;
        double waitTime = 0.2;
        addCommands(
                new ShootCargoAuton(index, shooter).withTimeout(1.5),
                new DriveDistance(driveSpeed, 30, drivetrain).andThen(new WaitCommand(waitTime)),
                new ExtendIntake(intake).withTimeout(0.2),
                new StopAll(shooter, index, intake));
    }
}
