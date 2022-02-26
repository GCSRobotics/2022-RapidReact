/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Add your docs here.
 */
public abstract class BaseController extends Joystick {
    public Button ButtonX;
    public Button ButtonY;
    public Button ButtonA;
    public Button ButtonB;
    public Button ButtonL1;
    public Button ButtonR1;
    public Button ButtonL2;
    public Button ButtonR2;
    public Button ButtonStickL;
    public Button ButtonStickR;
    public Button ButtonOptionL;
    public Button ButtonOptionR;

    protected BaseController(final int port) {
        super(port);
    }

    public abstract double GetAxis_LeftX();

    public abstract double GetAxis_LeftY();

    public abstract double GetAxis_RightX();

    public abstract double GetAxis_RightY();

    public abstract double GetTrigger_Left();

    public abstract double GetTrigger_Right();

    public static BaseController CreateInstance(ControllerType type, int port) {
        BaseController controller;
        switch (type) {
        case XBox:
            controller = new XBoxController(port);
            break;
        default:
            controller = new XBoxController(port);
            break;
        }
        return controller;
    }

    public void StartRumble() {
    }

    public void StopRumble() {
    }

    public int getLeftTriggerAxis() {
        return 0;
    }

}
