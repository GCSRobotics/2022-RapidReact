// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GroupCommands.ShootCargo;
import frc.robot.commands.GroupCommands.StopAll;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

public class SimpleAuton extends SequentialCommandGroup {
    public SimpleAuton(DriveSubsystem drivetrain, ShooterSub shooter, IndexSub index, IntakeSub intake) {
        double driveSpeed = 0.40;
        addCommands(
                // Backup
                new DriveDistance(driveSpeed, 52, drivetrain).withTimeout(1.5),
                // Shoot preloaded Ball
                new ShootCargo(index, shooter).withTimeout(1.5),
                new StopAll(shooter, index, intake, drivetrain));
    }
}
