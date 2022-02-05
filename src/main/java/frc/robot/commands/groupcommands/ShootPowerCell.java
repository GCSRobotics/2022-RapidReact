/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groupcommands;

import java.util.Date;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ShootPowerCell extends CommandBase {
  private IndexSub indexSub;
  private ShooterSub shooterSub;
  private IntakeSub intakeSub;
  private Date startTime;
  private double degrees;
  private double speed;

  public ShootPowerCell(ShooterSub shooter, IndexSub index, IntakeSub intake, double shooterDegree, double shooterSpeed) {
    shooterSub = shooter;
    indexSub = index;
    intakeSub = intake;
    degrees = shooterDegree;
    speed = shooterSpeed;
    addRequirements(indexSub);
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // degrees = SmartDashboard.getNumber("Shooter Degrees", 45);
    shooterSub.setShooterPosition(degrees);
    startTime = new Date();
    intakeSub.extendIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSub.RunShooter(speed);
    if ((new Date()).getTime() - startTime.getTime() > 500) {
      indexSub.indexBall();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSub.StopShooter();
    indexSub.StopIndex();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}