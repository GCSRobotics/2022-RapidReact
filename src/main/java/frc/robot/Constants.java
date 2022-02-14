// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    // Control Systems
    public final static int PCM = 1;  //Pneumatics

    // Game Controllers
    public final static int DriveJoystick = 0;
    public final static int OperatorJoystick = 1;
    
    // Motor Constants
    public final static int LeftFrontDriveMotor = 12;
    public final static int LeftRearDriveMotor = 13;
    public final static int RightFrontDriveMotor = 10;
    public final static int RightRearDriveMotor = 11;

    // Wheel Distance Calculations
    public final static double WheelDiameter = 6.00;
    public final static double DriveGearRatio = 10.71; // Tufbox Mini Default Ratio 10.71:1
    public final static double NeoEncoderCountsPerRev = 42;
    public final static double InchesPerMotorRotation = Math.PI * WheelDiameter / DriveGearRatio; // NeoEncoderCountsPerRev    


    // Shooter Constants
    public final static int TopShootingMotor = 21;
    public final static int BottomShootingMotor = 22;
    public static final double TopShooterConversionFactor = 0;
    public static final double BottomShooterConversionFactor = 0;


    // Intake Constants
    public final static int IntakeMotor = 31;
    public final static int IntakeExtendChannel = 32;
    public final static int IntakeRetractChannel = 33;

    

    // Index Constants
    public final static int FrontIndexMotor = 41;
    public final static int BackIndexMotor = 42;

}
