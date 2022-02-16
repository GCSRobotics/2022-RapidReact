// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.controllers.BaseController;
import frc.robot.controllers.ControllerType;

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
        //DriverControl.ButtonA.whenPressed(new RetractIntake(RobotContainer.intake));
    }

    public BaseController GetDriverControl() {
        return DriverControl;
    }

    public BaseController GetOperatorControl() {
        return OperatorControl;
    }
}
