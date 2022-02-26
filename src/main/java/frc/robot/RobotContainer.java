// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveSub.DriveWithController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

public class RobotContainer {

  // Subsystems
  public static DriveSubsystem driveSub = new DriveSubsystem();
  public static ShooterSub shootSub = new ShooterSub();
  public static IntakeSub intakeSub = new IntakeSub();
  public static IndexSub indexSub = new IndexSub();
  // Operator Interface
  private static OI oi = new OI();

  // Robot Container Constructor
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  public void setTeleopDefaultCommands() {
    driveSub.setDefaultCommand(new DriveWithController(driveSub, oi.GetDriverControl()));
  }

  //
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // An ExampleCommand will run in autonomous
  // return m_autoCommand;
  // }

  // Autonomous Commands
  // public Command GetAutonomousCommand() {
  // double WaitTime = 0.1;
  // double StraightSpeed = 0.40;
  // double TurnSpeed = 0.4;

  // // return new AutonomousDistance(drive);

  // // return new SequentialCommandGroup(
  // // new DriveDistance(StraightSpeed, 150, drive),
  // // new WaitCommand(WaitTime),
  // // new FindPowerCellA(drive));

  // // return new SequentialCommandGroup(
  // // new DriveDistance(StraightSpeed, 162, drive),
  // // new WaitCommand(WaitTime),
  // // new TurnDegreesGyro(TurnSpeed, -90, drive),
  // // new WaitCommand(WaitTime),
  // // new FindPowerCellB(drive));

  // //return new BarrelRacing(drive);

  // //return new BouncePath(drive);

  // //return new SlalomPath(drive);

  // // return new TwoBallCargoScore(driveSub , shootSub, indexSub, intakeSub);

  // }
}
