// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.GroupCommands.ShootCargo;
import frc.robot.commands.GroupCommands.StopAll;
import frc.robot.commands.IndexSub.IndexForward;
import frc.robot.commands.IntakeSub.ExtendIntake;
import frc.robot.commands.IntakeSub.IntakeForward;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

public class TwoBallCargoScore extends SequentialCommandGroup {

  DriveSubsystem m_drive;

  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a
   * specified distance, turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public TwoBallCargoScore(DriveSubsystem drivetrain, ShooterSub shooter, IndexSub index, IntakeSub intake) {
    m_drive = drivetrain;

    double driveSpeed = 0.40;
    double turnSpeed = 0.45;

    double waitTime = 0.2;
    addCommands(
     new ShootCargo(index, shooter),
     new ExtendIntake(intake),
     new IntakeForward(intake),
     new IndexForward(index),
     new DriveDistance(driveSpeed, 48, drivetrain).andThen(new WaitCommand(waitTime)),
     new ShootCargo(index, shooter).withTimeout(1.5),
     new DriveDistance(driveSpeed, 12, drivetrain),
     new StopAll(shooter, index, intake)
    // new TurnDegreesGyro(turnSpeed, 90, drivetrain).andThen(new WaitCommand(waitTime))
    //new DriveDistance(driveSpeed, 40, drivetrain).andThen(new WaitCommand(waitTime)),
    //new TurnDegreesGyro(turnSpeed, -180, drivetrain).andThen(new WaitCommand(waitTime)),
    //new DriveDistance(driveSpeed, 48, drivetrain).andThen(new WaitCommand(waitTime)),
    //new TurnDegreesGyro(turnSpeed, -270, drivetrain).andThen(new WaitCommand(waitTime)),
    // new DriveDistance(driveSpeed, 40, drivetrain).andThen(new WaitCommand(waitTime)),
    //new TurnDegreesGyro(turnSpeed, -360, drivetrain)
     

    );
  }
}
