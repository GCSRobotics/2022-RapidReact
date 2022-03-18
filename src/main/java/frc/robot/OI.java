// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controllers.BaseController;
import frc.robot.controllers.ControllerType;
import frc.robot.commands.IndexSub.IndexReverse;
import frc.robot.commands.IntakeSub.ExtendIntake;
import frc.robot.commands.IntakeSub.IntakeForward;
import frc.robot.commands.IntakeSub.IntakeReverse;
import frc.robot.commands.IntakeSub.RetractIntake;
import frc.robot.commands.ShooterSub.TurnShooterDegrees;
import frc.robot.commands.ClimbSub.*;
import frc.robot.commands.GroupCommands.ShootCargo;
import frc.robot.commands.GroupCommands.ShootCargoTwo;
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
        
        //Climb Controls
        DriverControl.DPadUp.whenHeld(new ExtendClimb(RobotContainer.climbSub));
        DriverControl.DPadRight.whenPressed(new ClimbOut(RobotContainer.climbSub));
        DriverControl.DPadDown.whenHeld(new RetractClimb(RobotContainer.climbSub));
        DriverControl.DPadLeft.whenPressed(new ClimbIn(RobotContainer.climbSub));


        // Operator buttons.
        OperatorControl.ButtonR1.whenHeld(new IntakeForward(RobotContainer.intakeSub));
        OperatorControl.ButtonL1.whenHeld(new IntakeReverse(RobotContainer.intakeSub));

        //OperatorControl.ButtonA.whenHeld(new IndexForward(RobotContainer.indexSub));
        //OperatorControl.ButtonX.whenHeld(new IndexReverse(RobotContainer.indexSub));

        // OperatorControl.ButtonX.whenHeld(new ShootCargo(RobotContainer.indexSub, RobotContainer.shootSub));
        OperatorControl.ButtonB.whenHeld(new ShootCargoTwo(RobotContainer.indexSub, RobotContainer.shootSub));

        OperatorControl.ButtonR2.whenHeld(new IndexForward(RobotContainer.indexSub, OperatorControl::GetTrigger_Right));
        OperatorControl.ButtonL2.whenHeld(new IndexReverse(RobotContainer.indexSub));

        OperatorControl.DPadLeft.whenPressed(new TurnShooterDegrees(RobotContainer.shootSub, 0));
        OperatorControl.DPadUp.whenPressed(new TurnShooterDegrees(RobotContainer.shootSub, 90));
        OperatorControl.DPadRight.whenPressed(new TurnShooterDegrees(RobotContainer.shootSub, 180));


    }

    public BaseController GetDriverControl() {
        return DriverControl;
    }

    public BaseController GetOperatorControl() {
        return OperatorControl;
    }
}
