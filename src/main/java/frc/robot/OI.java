// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controllers.BaseController;
import frc.robot.controllers.ControllerType;
import frc.robot.commands.GroupCommands.ShootCargo;
import frc.robot.commands.IndexSub.IndexReverse;
import frc.robot.commands.IntakeSub.ExtendIntake;
import frc.robot.commands.IntakeSub.IntakeForward;
import frc.robot.commands.IntakeSub.IntakeReverse;
import frc.robot.commands.IntakeSub.RetractIntake;
import frc.robot.commands.IndexSub.IndexForward;

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
        DriverControl.ButtonY.whenPressed(new ExtendIntake(RobotContainer.intakeSub));
        DriverControl.ButtonA.whenPressed(new RetractIntake(RobotContainer.intakeSub));

        // Operator buttons.
        OperatorControl.ButtonR2.whenHeld(new IntakeForward(RobotContainer.intakeSub));
        OperatorControl.ButtonR1.whenHeld(new IntakeReverse(RobotContainer.intakeSub));

        OperatorControl.ButtonL2.whenHeld(new IndexForward(RobotContainer.indexSub));
        OperatorControl.ButtonL1.whenHeld(new IndexReverse(RobotContainer.indexSub));

        OperatorControl.ButtonB.whenHeld(new ShootCargo(RobotContainer.indexSub, RobotContainer.shootSub));
    }

    public BaseController GetDriverControl() {
        return DriverControl;
    }

    public BaseController GetOperatorControl() {
        return OperatorControl;
    }
}
