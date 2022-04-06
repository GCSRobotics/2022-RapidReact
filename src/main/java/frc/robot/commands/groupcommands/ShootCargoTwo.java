// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroupCommands;

import java.util.Date;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.ShooterSub;

public class ShootCargoTwo extends CommandBase {
    /** Creates a new ShootCargo. */
    IndexSub indexSub;
    ShooterSub shooterSub;
    Date initime;
    private PIDController pidController = new PIDController(0.045, 0, 0.0025);
    private double speed = 0.1;

    public ShootCargoTwo(IndexSub index, ShooterSub shooter) {
        // Use addRequirements() here to declare subsystem dependencies.
        indexSub = index;
        shooterSub = shooter;
        addRequirements(indexSub);
        addRequirements(shooterSub);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initime = new Date();
        // Always align the turret so that limelight x = 0.0
        // or center of view
        pidController.setSetpoint(0.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
       // shooterSub.RunShooterRPM(8500);
       shooterSub.RunShooterRPM(7600);

        long timePassedMil = (new Date()).getTime() - initime.getTime();
        // Don't run the index until the shooter is up to speed
        if (timePassedMil > 800) {
            if (indexSub.CargoIndexed() && timePassedMil < 1000) {
                indexSub.StopIndex();
            } else {
                indexSub.RunIndex(0.3);
            }
        }

        shooterSub.alignTurret(pidController, speed);
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
