// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveSub;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.controllers.BaseController;
import frc.robot.subsystems.DriveSubsystem;

public class DriveWithController extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final BaseController driveController;

  double m_speedMultiplier = 0.7;
  double m_speedMutiplierNormal = 0.7;
  double m_speedMultiplierTurbo = 1.0;
  double m_rotationMultiplier = 0.6;

  /** Creates a new DriveWithController. */
  public DriveWithController(DriveSubsystem subsystem, BaseController baseController) {
    driveSubsystem = subsystem;
    driveController = baseController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Use Left Trigger to determine if Turbo Mode is on
    if (driveController.ButtonL1.get()) {
      m_speedMultiplier = m_speedMultiplierTurbo;
    } else {
      m_speedMultiplier = m_speedMutiplierNormal;
    }

    double forwardSpeed = driveController.GetAxis_LeftY() * m_speedMultiplier;
    double rotationSpeed = driveController.GetAxis_RightX() * m_rotationMultiplier;

   driveSubsystem.arcadeDrive(forwardSpeed,-rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
