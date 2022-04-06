// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterSub;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.ShooterSub;

public class AlignTurretToTarget extends CommandBase {
    IndexSub indexSub;
    ShooterSub shooterSub;
    private PIDController pidController = new PIDController(0.045, 0, 0.0035);
    private double speed = 0.15;

    /** Creates a new AlignTurretToTarget. */
    public AlignTurretToTarget(ShooterSub shooter) {
        // Use addRequirements() here to declare subsystem dependencies.
        // Use addRequirements() here to declare subsystem dependencies.
        shooterSub = shooter;
        addRequirements(shooterSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Always align the turret so that limelight x = 0.0
        // or center of view
        pidController.setSetpoint(0.0);
        pidController.setTolerance(2.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterSub.alignTurret(pidController, speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSub.StopTurret();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
