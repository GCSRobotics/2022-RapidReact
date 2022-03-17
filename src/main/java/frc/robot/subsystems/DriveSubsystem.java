// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    // Define Motors
    private CANSparkMax leftFrontMotor = new CANSparkMax(Constants.LeftFrontDriveMotor, MotorType.kBrushless);
    private CANSparkMax leftRearMotor = new CANSparkMax(Constants.LeftRearDriveMotor, MotorType.kBrushless);
    private CANSparkMax rightFrontMotor = new CANSparkMax(Constants.RightFrontDriveMotor, MotorType.kBrushless);
    private CANSparkMax rightRearMotor = new CANSparkMax(Constants.RightRearDriveMotor, MotorType.kBrushless);

    MotorControllerGroup leftMotorGroup = new MotorControllerGroup(leftFrontMotor, leftRearMotor);
    MotorControllerGroup rightMotorGroup = new MotorControllerGroup(rightFrontMotor, rightRearMotor);

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private final DifferentialDrive robotDrive;

    // private final ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS1);
    // Gyro Definitions
    AHRS ahrs;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        leftFrontMotor.setIdleMode(IdleMode.kCoast);
        leftRearMotor.setIdleMode(IdleMode.kCoast);
        rightFrontMotor.setIdleMode(IdleMode.kCoast);
        rightFrontMotor.setIdleMode(IdleMode.kCoast);

        leftFrontMotor.setInverted(true);
        leftRearMotor.setInverted(true);

        leftEncoder = leftFrontMotor.getEncoder();
        rightEncoder = rightFrontMotor.getEncoder();
        leftEncoder.setPositionConversionFactor(Constants.InchesPerMotorRotation);
        rightEncoder.setPositionConversionFactor(Constants.InchesPerMotorRotation);

        // gyro.calibrate();
        CreateNavXObject();
        resetEncoders();

        SmartDashboard.putNumber("LeftEncoder", leftEncoder.getPosition());
        SmartDashboard.putNumber("RightEncoder", rightEncoder.getPosition());
        // SmartDashboard.putNumber("Gyro", gyro.getAngle());
        SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

        robotDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);

    }

    private void CreateNavXObject() {
        try {
            /* Communicate w/navX-MXP via the MXP SPI Bus. */
            /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
            /*
             * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
             * details.
             */
            ahrs = new AHRS(SerialPort.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LeftEncoder", leftEncoder.getPosition());
        SmartDashboard.putNumber("RightEncoder", rightEncoder.getPosition());
        // SmartDashboard.putNumber("Gyro", gyro.getAngle());
        System.out.print(leftEncoder.getPosition());
        SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

        // This method will be called once per scheduler run
    }

    // ********************************************
    // DRIVE - Methods to Drive robot
    // ********************************************
    public void arcadeDrive(double speedAxis, double rotationAxis) {
        robotDrive.arcadeDrive(speedAxis, rotationAxis, true);
    }

    public void arcadeDrive(double speedAxis, double rotationAxis, boolean squared) {
        robotDrive.arcadeDrive(speedAxis, rotationAxis, squared);
    }

    public void setMaxDriveSpeed(double maxSpeed) {
        robotDrive.setMaxOutput(maxSpeed);
    }

    public void stop() {
        robotDrive.arcadeDrive(0, 0);
    }

    // ********************************************
    // ENCODER - Methods to get Encoder Readings
    // ********************************************
    public void resetEncoders() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
    }

    public double getLeftDistanceInch() {
        return leftEncoder.getPosition();
    }

    public double getRightDistanceInch() {
        return rightEncoder.getPosition();
    }

    public double getAverageDistanceInch() {
        return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
    }

    // ********************************************
    // Gyro - Methods to get Gyro Readings
    // ********************************************
    public double getGyroAngle() {
        // return gyro.getAngle();
        return ahrs.getAngle();
    }

    public double getGyroRate() {
        // return gyro.getRate();
        return ahrs.getRate();
    }

    public void reset() {
    }
}
