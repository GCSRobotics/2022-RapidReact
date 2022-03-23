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
        rightRearMotor.setIdleMode(IdleMode.kCoast);

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
        SmartDashboard.putNumber("IMU_Angle", ahrs.getAngle());


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
            // ahrs.reset();
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
        SmartDashboard.putNumber("IMU_Angle", ahrs.getAngle());
        UpdateNavXDashboard();
        

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
    public double getGyroYaw() {
        // return gyro.getAngle();
        return ahrs.getYaw();
    }

    public double getGyroRate() {
        // return gyro.getRate();
        return ahrs.getRate();
    }

    public void reset() {
        ahrs.reset();
        
    }

    public void StopAll(){
        leftMotorGroup.set(0.0);
        rightMotorGroup.set(0.0);
    }

    private void UpdateNavXDashboard() {
        // * Display 6-axis Processed Angle Data */
        SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
        SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
        SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
        SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());
      
        /* Display tilt-corrected, Magnetometer-based heading (requires */
        /* magnetometer calibration to be useful) */
      
        SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());
      
        /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
        SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());
      
        /* These functions are compatible w/the WPI Gyro Class, providing a simple */
        /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP */
      
        SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
        SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());
      
        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
      
        SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
        SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
        SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
        SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());
      
        /* Display estimates of velocity/displacement. Note that these values are */
        /* not expected to be accurate enough for estimating robot position on a */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially */
        /* double (displacement) integration. */
      
        SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
        SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
        SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
        SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());
      
        /* Display Raw Gyro/Accelerometer/Magnetometer Values */
        /* NOTE: These values are not normally necessary, but are made available */
        /* for advanced users. Before using this data, please consider whether */
        /* the processed data (see above) will suit your needs. */
      
        SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
        SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
        SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
        SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
        SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
        SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
        SmartDashboard.putNumber("RawMag_X", ahrs.getRawMagX());
        SmartDashboard.putNumber("RawMag_Y", ahrs.getRawMagY());
        SmartDashboard.putNumber("RawMag_Z", ahrs.getRawMagZ());
        SmartDashboard.putNumber("IMU_Temp_C", ahrs.getTempC());
      
        /* Omnimount Yaw Axis Information */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
        AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
        SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
        SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());
      
        /* Sensor Board Information */
        SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());
      
        /* Quaternion Data */
        /* Quaternions are fascinating, and are the most compact representation of */
        /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
        /* from the Quaternions. If interested in motion processing, knowledge of */
        /* Quaternions is highly recommended. */
        SmartDashboard.putNumber("QuaternionW", ahrs.getQuaternionW());
        SmartDashboard.putNumber("QuaternionX", ahrs.getQuaternionX());
        SmartDashboard.putNumber("QuaternionY", ahrs.getQuaternionY());
        SmartDashboard.putNumber("QuaternionZ", ahrs.getQuaternionZ());
      
        /* Connectivity Debugging Support */
        SmartDashboard.putNumber("IMU_Byte_Count", ahrs.getByteCount());
        SmartDashboard.putNumber("IMU_Update_Count", ahrs.getUpdateCount());
      }
    
}
