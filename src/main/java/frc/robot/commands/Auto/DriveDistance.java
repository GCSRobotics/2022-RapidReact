// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;

public class DriveDistance extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double m_distance;
  private final double m_speed;
  private final PIDController m_pidController = new PIDController(0.045, 0, 0.0025);

  /**
   * Creates a new DriveDistance. This command will drive your your robot for a
   * desired distance at a desired speed.
   *
   * @param speed  The speed at which the robot will drive
   * @param inches The number of inches the robot will drive
   * @param drive  The drivetrain subsystem on which this command will run
   */
  public DriveDistance(double speed, double inches, DriveSubsystem drive) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    addRequirements(drive);
    
  }

  // Called once when the scheduler loads the command.
  @Override
  public void initialize() {
   // m_drive.arcadeDrive(0, 0);
    m_drive.resetEncoders();
    m_pidController.setSetpoint(m_distance);
    // Add a tolerence to the PID loop to allow the "atSetpoint()" method to
    // function within a range of values. Read the 'Specifying and Checking
    // Tolerance' section on this page for more details //
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html
    m_pidController.setTolerance(0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Drive Straight Running");
    
    // Use the MathUtil.Clamp() function to ensure the that motor speed (i.e.
    // 'output' below) variable never goes outside the +/- range of the m_Speed
    // variable passed into the command.
    // (-m_Speed <= output <= m_Speed). See the "Clamping Controller Output" section
    // of the following page
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html
    double output = m_pidController.calculate(m_drive.getAverageDistanceInch());
    double outputC = MathUtil.clamp(output, -m_speed, m_speed);

    SmartDashboard.putNumber("avgD", m_drive.getAverageDistanceInch()) ;
    SmartDashboard.putNumber("pitO", output) ;
   
    m_drive.arcadeDrive(outputC, 0, false);
   // m_drive.arcadeDrive(outputC, 0);

  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0, false);
  }

  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    
    return m_pidController.atSetpoint();// Math.abs(m_drive.getAverageDistanceInch()) >= m_distance*2;
  }
}
