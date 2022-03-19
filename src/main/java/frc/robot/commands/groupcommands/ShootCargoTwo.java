// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GroupCommands;

import java.util.Date;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.ShooterSub;

public class ShootCargoTwo extends CommandBase {
    /** Creates a new ShootCargo. */
    IndexSub indexSub;
    ShooterSub shooterSub;
    Date initime;
    private PIDController m_pidController = new PIDController(0.045, 0, 0.0025);
    private double m_speed = 0.1;



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
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterSub.RunShooterRPM(8300);
        long timePassedMil = (new Date()).getTime() - initime.getTime();
        // Don't run the index until the shooter is up to speed
        if (timePassedMil > 500) {
            if (indexSub.CargoIndexed() && timePassedMil < 1000) {
                indexSub.StopIndex();
            } else {
                indexSub.RunIndex(0.3);
            }
        }

        if (shooterSub.LimelightTargetFound()) {
            double setpoint = shooterSub.getLimelightXPos() + shooterSub.getTurretDegrees();
            if (setpoint > 180) {
                setpoint = 180;  
            }
            if (setpoint < 0) {
                setpoint = 0;
            }
            m_pidController.setSetpoint(setpoint);
            
            double output = m_pidController.calculate(shooterSub.getTurretDegrees());
            double outputC = MathUtil.clamp(output, -m_speed, m_speed);
        
            SmartDashboard.putNumber("TargetDegrees", setpoint);
            SmartDashboard.putNumber("PID output", output);
            SmartDashboard.putNumber("PID outputC", outputC);
        
            shooterSub.RunTurret(outputC);
        

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
