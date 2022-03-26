// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Auton.SimpleAuton;
import frc.robot.commands.Auton.ThreeBall;
import frc.robot.commands.DriveSub.DriveWithController;
import frc.robot.subsystems.*;

public class RobotContainer {

  // Subsystems
  public static DriveSubsystem driveSub = new DriveSubsystem();
  public static ShooterSub shootSub = new ShooterSub();
  public static IntakeSub intakeSub = new IntakeSub();
  public static IndexSub indexSub = new IndexSub();
  public static ClimbSub climbSub = new ClimbSub();
  // Operator Interface
  private static OI oi = new OI();
  SendableChooser<CommandBase> auton_chooser = new SendableChooser<>();

  // Robot Container Constructor
  public RobotContainer() {
    // Bring up the default camera server for the RIO camera
    CameraServer.startAutomaticCapture(0);
    // CameraServer.startAutomaticCapture(1);
    SmartDashboard.putNumber(Constants.TurretStartPositionPrompt, Constants.TurretStartPositionDefault);
    SmartDashboard.putData("DriveSub", driveSub);

    // Add commands to the autonomous command chooser
    auton_chooser.setDefaultOption("Three Ball Auto", new ThreeBall(driveSub, indexSub, shootSub, intakeSub));
    auton_chooser.addOption("Simple Auto", new SimpleAuton(driveSub, shootSub, indexSub, intakeSub));

    // Put the chooser on the dashboard
    SmartDashboard.putData(auton_chooser);
  }

  //
  public void AutonomousInit() {
    // double turretPos =
    // SmartDashboard.getNumber(Constants.TurretStartPositionPrompt,
    // Constants.TurretStartPositionDefault);
    // shootSub.setShooterPosition(turretPos);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // Autonomous Commands
  public CommandBase GetAutonomousCommand() {
    return auton_chooser.getSelected();
    // return new SimpleAuton(driveSub, shootSub, indexSub, intakeSub);
  }

  // Teleop Iinitialization methods
  public void TeleOpInit() {
    // double turretPos =
    // SmartDashboard.getNumber(Constants.TurretStartPositionPrompt,
    // Constants.TurretStartPositionDefault);
    // shootSub.setShooterPosition(turretPos);

    this.setTeleopDefaultCommands();
  }

  private void setTeleopDefaultCommands() {
    // Drive with controllers is always the drive sub default.
    driveSub.setDefaultCommand(new DriveWithController(driveSub, oi.GetDriverControl()));

    // Use a IndexSub default command to allow the sensors to turn on/off the index
    // motors.
    // indexSub.setDefaultCommand(new IndexAutomatic(indexSub));
  }

}
