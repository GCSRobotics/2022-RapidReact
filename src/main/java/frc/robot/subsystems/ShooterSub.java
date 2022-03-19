// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSub extends SubsystemBase {
    // PID loop Values for velocity target
    private int PidLoopIdx = 0;
    private int timeoutMS = 30; // provides for an DS error is config settings timeout

    // Shooter PID Gains (From Atherton)
    private static final double kP = 0.06;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.0465;

    // Shooter Motor Properties
    private TalonFX ShootingMotor = new TalonFX(Constants.ShootingMotor);

    // Turret Motor Properties
    private CANSparkMax TurretMotor = new CANSparkMax(Constants.TurretMotor, MotorType.kBrushless);
    private RelativeEncoder TurretEncoder;

    private NetworkTable limelight;
    private NetworkTableEntry limelightTX;
    private NetworkTableEntry limelightTV;

    /** Creates a new ShooterSub. */
    public ShooterSub() {
        initShooterMotor();
        initTurretMotor();

        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        limelightTX = limelight.getEntry("tx");
        limelightTV = limelight.getEntry("tv");
    
        SmartDashboard.putData("ShooterSub", this);

    }

    private void initShooterMotor() {
        /* Factory Default all hardware to prevent unexpected behaviour */
        ShootingMotor.configFactoryDefault();

        ShootingMotor.setInverted(true);

        /* Config neutral deadband to be the smallest possible */
        ShootingMotor.configNeutralDeadband(0.001);

        /* Config sensor used for Primary PID [Velocity] */
        ShootingMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PidLoopIdx, timeoutMS);

        /* Config the peak and nominal outputs */
        ShootingMotor.configNominalOutputForward(0, timeoutMS);
        ShootingMotor.configNominalOutputReverse(0, timeoutMS);
        ShootingMotor.configPeakOutputForward(1, timeoutMS);
        ShootingMotor.configPeakOutputReverse(-1, timeoutMS);

        /* Config the Velocity closed loop gains in slot0 */
        ShootingMotor.config_kF(PidLoopIdx, kF, timeoutMS);
        ShootingMotor.config_kP(PidLoopIdx, kP, timeoutMS);
        ShootingMotor.config_kI(PidLoopIdx, kI, timeoutMS);
        ShootingMotor.config_kD(PidLoopIdx, kD, timeoutMS);
    }

    private void initTurretMotor() {
        TurretMotor.setInverted(true);
        TurretEncoder = TurretMotor.getEncoder();
        TurretEncoder.setPositionConversionFactor(Constants.TurretRevolutionsPerDegree);
        ResetTurretPosition(Constants.TurretStartPositionDefault);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("TurretPosition", this.getTurretDegrees());
        SmartDashboard.putNumber("EncoderConversionFactor", TurretEncoder.getPositionConversionFactor());
        SmartDashboard.putNumber("ShooterOutputPct", ShootingMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("ShooterVelocityNative", ShootingMotor.getSelectedSensorVelocity(PidLoopIdx));
    }

    public void ResetTurretPosition(double degrees) {
        TurretEncoder.setPosition(degrees);
    }

    public void RunShooter(double speed) {
        ShootingMotor.set(ControlMode.PercentOutput, speed);
    }

    public void RunShooterRPM(double targetRPM) {
        SmartDashboard.putNumber("ShooterTargetRPM", targetRPM);

        // Determine the target velocity in TalonFX units using 2:1 gear ratio
        double targetVelocity_FalconUnits = RPM2Falcon(targetRPM, 2.0);

        ShootingMotor.set(TalonFXControlMode.Velocity, targetRPM);
    }

    public double GetShooterRPM() {
        // Need to add some calculations here.
        return ShootingMotor.getSelectedSensorVelocity(PidLoopIdx);
    }

    private double RPM2Falcon(double RPM, double gearRatio) {
        /**
         * Convert RPM to units / 100ms.
         * i.e. 2000 RPM * 2048 Units/Rev / 600 100ms/min
         * velocity setpoint is in units/100ms (600 * 100ms) = 6000ms = 6 sec
         * https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/VelocityClosedLoop/src/main/java/frc/robot/Robot.java#L141
         */
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    public void StopShooter() {
        ShootingMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void setShooterPosition(double degrees) {
    }

    // public void TurnTurret(double speed) {
    // float Kp = -0.1f;
    // float min_command = 0.05f;

    // NetworkTableEntry tx = table.getEntry("tx");
    // double targetOffsetAngle_Vertical = tx.getDouble(0.0);

    // double heading_error = -targetOffsetAngle_Vertical;
    // double steering_adjust = 0.0f;
    // if (targetOffsetAngle_Vertical > 1.0) {
    // steering_adjust = Kp * heading_error - min_command;
    // } else if (targetOffsetAngle_Vertical < 1.0) {
    // steering_adjust = Kp * heading_error + min_command;
    // }
    // // TurretMotor.set(steering_adjust);
    // }

    public void StopTurret() {
        TurretMotor.set(0.0);
    }

    public void RunTurret(double speed) {
        TurretMotor.set(speed);
    }

    public double getTurretDegrees() {
        return TurretEncoder.getPosition();
    }

    public double getLimelightXPos() {
        return limelightTX.getDouble(0);
    }

    public boolean LimelightTargetFound() {
        return (limelightTV.getDouble(0) == 1.0);
    }
}