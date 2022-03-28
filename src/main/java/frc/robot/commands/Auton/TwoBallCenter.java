// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.GroupCommands.*;
import frc.robot.commands.IndexSub.*;
import frc.robot.commands.IntakeSub.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallCenter extends SequentialCommandGroup {
  /** Creates a new TwoBallCenter. */
  public TwoBallCenter(DriveSubsystem drivetrain, IndexSub index, ShooterSub shooter, IntakeSub intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double driveSpeed = 0.45;
    double turnSpeed = 0.35;
    double waitTime = 0.2;
    addCommands(
        // Backup
        new DriveDistance(driveSpeed, 52, drivetrain),
        // Shoot preloaded Ball
        new ShootCargo(index, shooter).withTimeout(1.5),
        // Turn and Get Ball 2
        new TurnDegreesGyro(turnSpeed, -90, drivetrain),
        new ExtendIntake(intake).withTimeout(0.2),
        new ParallelCommandGroup(
            new IntakeForward(intake),
            new IndexForward(index),
            new DriveDistance(driveSpeed, -12, drivetrain).andThen(new WaitCommand(waitTime))).withTimeout(1),
        new IndexReverse(index).withTimeout(0.2),
        new TurnDegreesGyro(turnSpeed, 90, drivetrain),
        new ShootCargo(index, shooter).withTimeout(1.5),
        new StopAll(shooter, index, intake, drivetrain)
    );
  }
}
