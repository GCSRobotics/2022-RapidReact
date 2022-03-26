// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnDegreesGyro extends CommandBase {
    private final DriveSubsystem m_drive;
    private final double m_degrees;
    private final double m_speed;
    private final PIDController m_pidController = new PIDController(0.25, 0.9, 0.1);

    /**
     * Creates a new TurnDegrees. This command will turn your robot for a desired
     * rotation (in
     * degrees) and rotational speed.
     *
     * @param speed   The speed which the robot will drive. Negative is in reverse.
     * @param degrees Degrees to turn. Leverages gyro to compare distance.
     * @param drive   The drive subsystem on which this command will run
     */
    public TurnDegreesGyro(double speed, double degrees, DriveSubsystem drive) {
        m_degrees = degrees;
        m_speed = speed;
        m_drive = drive;
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Set motors to stop, read encoder values for starting point
        // m_Drive.arcadeDrive(0, 0);
        m_pidController.setSetpoint(m_degrees);
        m_drive.reset();
        m_pidController.setTolerance(0.35);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double output = m_pidController.calculate(m_drive.getGyroAngle());
        double outputC = MathUtil.clamp(output, -m_speed, m_speed);

        // if(outputC < 0.1)
        // {
        // outputC = 0.1;
        // }
        m_drive.arcadeDrive(0, -outputC);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.arcadeDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        // double gyroAngle = m_Drive.getGyroAngle();
        // return gyroAngle >= m_Degrees;

        return m_pidController.atSetpoint();

    }

}
