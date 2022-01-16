// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class OI {
    private XboxController DriverControl;
    private XboxController OperatorControl;

    public OI() {
        DriverControl = new XboxController(Constants.DriveJoystick);
        OperatorControl = new XboxController(Constants.OperatorJoystick);
    }

    public XboxController GetDriverControl() {
        return DriverControl;
    }

    public XboxController GetOperatorControl() {
        return OperatorControl;
    }
}
