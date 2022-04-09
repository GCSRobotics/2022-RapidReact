// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroupCommands;

import java.util.Date;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.ShooterSub;

public class SpitCargo extends CommandBase {
    /** Creates a new ShootCargo. */
    IndexSub indexSub;
    ShooterSub shooterSub;
    Date initime;
    private PIDController pidController = new PIDController(0.045, 0, 0.0025);
    private double speed = 0.1;

    public SpitCargo(IndexSub index, ShooterSub shooter) {
        // Use addRequirements() here to declare subsystem dependencies.
        indexSub = index;
        shooterSub = shooter;
        addRequirements(indexSub);
        addRequirements(shooterSub);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Always align the turret so that limelight x = 0.0
        // or center of view
        pidController.setSetpoint(0.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterSub.RunShooterRPM(3000);
        indexSub.RunIndex(.3);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSub.StopShooter();
        indexSub.StopIndex();
        shooterSub.StopTurret();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
