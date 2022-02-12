// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import frc.robot.commands.Auto.DriveDistance;
import frc.robot.commands.DriveSub.TurnDegreesGyro;
import frc.robot.commands.groupcommands.ShootPowerCell;
import frc.robot.commands.ShootCargo;
import frc.robot.subsystems.DriveSub.DriveSubsystem;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousDistance extends SequentialCommandGroup {

  DriveSubsystem m_drive;

  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a
   * specified distance, turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(DriveSubsystem drivetrain, ShooterSub shooter, IndexSub index, IntakeSub intake) {
    m_drive = drivetrain;

    double driveSpeed = 0.40;
    double turnSpeed = 0.45;

    double waitTime = 0.2;
    addCommands(
       // new DriveDistance(driveSpeed, 36, drivetrain).andThen(new WaitCommand(waitTime))
   // new TurnDegreesGyro(turnSpeed, 180, drivetrain).andThen(new WaitCommand(waitTime))
    // new DriveDistance(driveSpeed, -40, drivetrain).andThen(new WaitCommand(waitTime))
    
    // new DriveDistance(driveSpeed, 40, drivetrain).andThen(new WaitCommand(waitTime))
    // new TurnDegreesGyro(turnSpeed, 180, drivetrain).andThen(new WaitCommand(waitTime))
    // new DriveDistance(driveSpeed, 40, drivetrain)
    

     new DriveDistance(driveSpeed, 48, drivetrain).andThen(new WaitCommand(waitTime)),
     new ShootPowerCell(shooter, index, intake, 0.25, 0.75).withTimeout(1.5),
     new TurnDegreesGyro(turnSpeed, 90, drivetrain).andThen(new WaitCommand(waitTime))
    //new DriveDistance(driveSpeed, 40, drivetrain).andThen(new WaitCommand(waitTime)),
    //new TurnDegreesGyro(turnSpeed, -180, drivetrain).andThen(new WaitCommand(waitTime)),
    //new DriveDistance(driveSpeed, 48, drivetrain).andThen(new WaitCommand(waitTime)),
    //new TurnDegreesGyro(turnSpeed, -270, drivetrain).andThen(new WaitCommand(waitTime)),
    // new DriveDistance(driveSpeed, 40, drivetrain).andThen(new WaitCommand(waitTime)),
    //new TurnDegreesGyro(turnSpeed, -360, drivetrain)
     

    );
  }
}
