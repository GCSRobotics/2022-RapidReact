// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterSub;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSub;

public class TurnShooterDegrees extends CommandBase {
  private ShooterSub shooterSub;
  private double TargetDegrees;
  private final PIDController m_pidController = new PIDController(0.045, 0, 0.0025);
  private double m_speed = 0.1;

  /** Creates a new TurnShooterDegrees. */
  public TurnShooterDegrees(ShooterSub shooter, double Degree) {
    shooterSub = shooter;
    TargetDegrees = Degree;
    addRequirements(shooter);
    m_pidController.setTolerance(0.5);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.setSetpoint(TargetDegrees);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = m_pidController.calculate(shooterSub.getTurretDegrees());
    double outputC = MathUtil.clamp(output, -m_speed, m_speed);

    SmartDashboard.putNumber("TargetDegrees", TargetDegrees);
    SmartDashboard.putNumber("PID output", output);
    SmartDashboard.putNumber("PID outputC", outputC);
    SmartDashboard.putBoolean("Turret AtSetpoint", m_pidController.atSetpoint());

    shooterSub.RunTurret(outputC);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSub.StopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }
}
