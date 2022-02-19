// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.wpilibj.XboxController;

import frc.robot.controllers.BaseController;
import frc.robot.controllers.ControllerType;
import frc.robot.commands.LoadCargo;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.IntakeForward;
import frc.robot.commands.Intake.IntakeReverse;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.groupcommands.ShootCargo;
import frc.robot.commands.indexSub.RunIndex;
import frc.robot.controllers.BaseController;
import frc.robot.controllers.ControllerType;
import frc.robot.controllers.XBoxController;


/** Add your docs here. */
public class OI {
    private BaseController DriverControl;
    private BaseController OperatorControl;

    public OI() {
        DriverControl = BaseController.CreateInstance(ControllerType.XBox, Constants.DriveJoystick);
        OperatorControl = BaseController.CreateInstance(ControllerType.XBox, Constants.OperatorJoystick);
        ButtonActionInit();
    }

    private void ButtonActionInit() {
    DriverControl.ButtonA.whenPressed(new RetractIntake(RobotContainer.intakeSub));
    DriverControl.ButtonY.whenPressed(new ExtendIntake(RobotContainer.intakeSub));
    DriverControl.ButtonR1.whenHeld(new IntakeForward(RobotContainer.intakeSub));
    DriverControl.ButtonL1.whenHeld(new IntakeReverse(RobotContainer.intakeSub));
    //DriverControl.ButtonB.whenPressed(new ShootPowerCell(RobotContainer.indexSub, RobotContainer.shootSub, RobotContainer.intakeSub, 45).withTimeout(3));
    DriverControl.ButtonX.whileHeld(new RunIndex(RobotContainer.indexSub));
    //DriverControl.ButtonR2.whileHeld(new ShootCargo(RobotContainer.shootSub));

    // Operator buttons.
    OperatorControl.ButtonY.whenPressed(new ExtendIntake(RobotContainer.intakeSub));
    OperatorControl.ButtonA.whenPressed(new RetractIntake(RobotContainer.intakeSub));
    OperatorControl.ButtonR1.whenHeld(new IntakeForward(RobotContainer.intakeSub));
    OperatorControl.ButtonL1.whenHeld(new IntakeReverse(RobotContainer.intakeSub));
    OperatorControl.ButtonX.whileHeld(new RunIndex(RobotContainer.indexSub));
    // OperatorControl.ButtonB.whenPressed(new ShootPowerCell(RobotContainer.indexSub, RobotContainer.shootSub, RobotContainer.intakeSub, 45).withTimeout(3));
    //OperatorControl.ButtonR2.whileHeld(new ShootCargo(RobotContainer.shootSub));
    }

    public BaseController GetDriverControl() {
        return DriverControl;
    }

    public BaseController GetOperatorControl() {
        return OperatorControl;
    }
}
