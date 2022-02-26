// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controllers.BaseController;
import frc.robot.controllers.ControllerType;
import frc.robot.commands.Intake.ExtendIntake;
import frc.robot.commands.Intake.IntakeForward;
import frc.robot.commands.Intake.IntakeReverse;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.groupcommands.ShootCargo;
import frc.robot.commands.indexSub.ReverseIndex;
import frc.robot.commands.indexSub.RunIndex;

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

        // Operator buttons.
        OperatorControl.ButtonR1.whenHeld(new IntakeForward(RobotContainer.intakeSub));
        OperatorControl.ButtonL1.whenHeld(new IntakeReverse(RobotContainer.intakeSub));
        //  OperatorControl.ButtonL1.whenHeld(new RunIndex(RobotContainer.indexSub));
        //  OperatorControl.ButtonR1.whenHeld(new ReverseIndex(RobotContainer.indexSub));
        OperatorControl.ButtonB
                .whenHeld(new ShootCargo(RobotContainer.indexSub, RobotContainer.shootSub));

    }

    public BaseController GetDriverControl() {
        return DriverControl;
    }

    public BaseController GetOperatorControl() {
        return OperatorControl;
    }
}
