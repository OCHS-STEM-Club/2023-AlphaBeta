// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDriveStraight;

public class DrivetrainSubsystem extends SubsystemBase {

  private CANSparkMax leftDrive1;
  private CANSparkMax leftDrive2;
  private MotorControllerGroup leftDriveGroup;
  private CANSparkMax rightDrive1;
  private CANSparkMax rightDrive2;
  private MotorControllerGroup rightDriveGroup;
  private DifferentialDrive drive;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private double creepSpeed = 1;

  private SparkMaxPIDController drivetrainPIDLeft;
  private SparkMaxPIDController drivetrainPIDRight;

  private double pidReference = 0;

  private final AHRS navX = new AHRS();
  private double navXGetHeading;

  private PIDController turningPIDController = new PIDController(0, 0, 0);
  private PIDController drivingPIDController = new PIDController(0, 0, 0);

  private double turnSetpointDegree = 0;

  private double turningPIDMax = 0.75;
  private double turningPIDMin = -0.75;

  private double drivingPValue;
  private double drivingIValue;
  private double drivingDValue;

  private double driveSetpointLength;

  /** Creates a new Drivetrain. */
  public DrivetrainSubsystem() {
    // Set up left drivetrain motors
    leftDrive1 = new CANSparkMax(Constants.DriveTrain.kleftDrive1Id, MotorType.kBrushless);
    leftDrive2 = new CANSparkMax(Constants.DriveTrain.kleftDrive2Id, MotorType.kBrushless);
    leftDrive1.setOpenLoopRampRate(Constants.DriveTrain.ksetOpenLoopRampRate);
    leftDrive2.setOpenLoopRampRate(Constants.DriveTrain.ksetOpenLoopRampRate);

    leftDriveGroup = new MotorControllerGroup(leftDrive1, leftDrive2);
    leftDriveGroup.setInverted(false);

    // Set up right drivetrain motors
    rightDrive1 = new CANSparkMax(Constants.DriveTrain.krightDrive1Id, MotorType.kBrushless);
    rightDrive2 = new CANSparkMax(Constants.DriveTrain.krightDrive2Id, MotorType.kBrushless);
    rightDrive1.setOpenLoopRampRate(Constants.DriveTrain.ksetOpenLoopRampRate);
    rightDrive2.setOpenLoopRampRate(Constants.DriveTrain.ksetOpenLoopRampRate);
    rightDriveGroup = new MotorControllerGroup(rightDrive1, rightDrive2);
    rightDriveGroup.setInverted(true);

    // Set up Differential Drive
    drive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);

    leftEncoder = leftDrive1.getEncoder();
    rightEncoder = rightDrive1.getEncoder();

    // compute conversion from encoder units to meters
    double conversionFactor = ((1 / Constants.DriveTrain.kdriveEncoderNativeUnitsPerRev)
        / Constants.DriveTrain.kdriveGearRatio) *
        (Math.PI * Constants.DriveTrain.kwheelDiameterInches) * Constants.DriveTrain.kinchesToMeters;

    // Set conversion factor for both encoders
    leftEncoder.setPositionConversionFactor(conversionFactor);
    rightEncoder.setPositionConversionFactor(conversionFactor);

    drivetrainPIDLeft = leftDrive1.getPIDController();
    drivetrainPIDRight = rightDrive1.getPIDController();

    //drivetrainPIDLeft.setP(2500);
    //drivetrainPIDRight.setP(2500);

    //turningPIDController.enableContinuousInput(0, 360);
    turningPIDController.disableContinuousInput();
    turningPIDController.setPID(0.018, 0.001, 0.00772); //setPID(0.04, 0.0025, 0.0175);
    turningPIDController.setTolerance(3);

    drivingPIDController.setPID(0.3, 0, 0);


    

    // SmartDashboard.putNumber("turning P", drivingPValue);
    // SmartDashboard.putNumber("turning I", drivingIValue);
    // SmartDashboard.putNumber("turning D", drivingDValue);
    // SmartDashboard.putNumber("turn setpoint", driveSetpointLength);

  //   pidReference = SmartDashboard.getNumber("PID Set Reference", 0);
  //  drivetrainPIDLeft.setReference(pidReference, CANSparkMax.ControlType.kPosition);


    navX.calibrate();
    navX.reset();
    navX.zeroYaw();


  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Left Encoder", getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoder", getRightEncoderDistance());

    SmartDashboard.putData(drive);

    //System.out.println("NavX Heading" + navXGetHeading);

    SmartDashboard.putNumber("NavX Roll" , navX.getRoll());

    SmartDashboard.putNumber("NavX Heading", navX.getYaw());

    System.out.println(getLeftEncoderDistance());

    //System.out.println(leftEncoder.getPosition());

    //System.out.println(navX.getRoll());

    //System.out.println(navX.getYaw());
    // System.out.println(leftDrive1.getOutputCurrent());

    // driveSetpointLength = SmartDashboard.getNumber("turn setpoint", 0);
    // drivingPValue = SmartDashboard.getNumber("turning P", 0);
    // drivingIValue = SmartDashboard.getNumber("turning I", 0);
    // drivingDValue = SmartDashboard.getNumber("turning D", 0);

    // drivingPIDController.setPID(drivingPValue, drivingIValue, drivingDValue);

    // drivingPIDController.setSetpoint(turnSetpointDegree);
    // double drivingPIDOutput = turningPIDController.calculate(leftEncoder.getPosition());

    // Clamping turning PID value between min and max outputs //
    // drivingPIDOutput = Math.max(drivingPIDOutput, turningPIDMin); 
    // drivingPIDOutput = Math.min(drivingPIDOutput, turningPIDMax);

    // drive.arcadeDrive(drivingPIDOutput, 0);



    // double xStickValue = RobotContainer.m_driverController.getRawAxis(4) *
    // Constants.DriveTrain.kspeedMultiplier * creepSpeed;
    // double yStickValue = -RobotContainer.m_driverController.getRawAxis(1) *
    // Constants.DriveTrain.kspeedMultiplier * creepSpeed;
    // This method will be called once per scheduler run
  }

  public void driveWithXbox() {
    if (RobotContainer.m_driverController.getRawAxis(2) == 1) {
      creepSpeed = 0.7;
    } else if (RobotContainer.m_driverController.getRawAxis(3) == 1) {
      creepSpeed = 1;
    } else {
      creepSpeed = 1;
    }
    // System.out.printf("Controller: forward: %f, turn: %f\n",
    // xboxController.getLeftY(), xboxController.getRightX());

    // drive.arcadeDrive(
    //     RobotContainer.m_driverController.getRawAxis(1) * Constants.DriveTrain.kspeedMultiplier * creepSpeed,
    //     RobotContainer.m_driverController.getRawAxis(4) * Constants.DriveTrain.kspeedMultiplier * creepSpeed);

  

  }

  // public void resetEncoders() {
  //   leftEncoder.setPosition(0);
  //   rightEncoder.setPosition(0);
  // }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public void auto(double distance, double speed) {
    if (leftEncoder.getPosition() > distance) {
      drive.arcadeDrive(-speed, 0.0);
    } else if (leftEncoder.getPosition() <= distance) {
      drive.arcadeDrive(0.0, 0.0);
    }
  }


  public void setEncodersToZero() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public double getLeftEncoderDistance() {
    return leftEncoder.getPosition();
  }

  public double getRightEncoderDistance() {
    return rightEncoder.getPosition();
  }

  public void setDrivetrainSpeed(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation, false);
  }

  public void subclassTurn(double turnValue, double moveValue) {
    drive.arcadeDrive(turnValue, moveValue, false);
  }

  public void resetNavX() {
    navX.calibrate();
    navX.reset();
    navX.zeroYaw();
  }

  public void turnWithPID(double turnSetpointDegree) {
    turningPIDController.setSetpoint(turnSetpointDegree);
    double turningPIDOutput = turningPIDController.calculate(navX.getAngle());

    // Clamping turning PID value between min and max outputs //
    turningPIDOutput = Math.max(turningPIDOutput, turningPIDMin); 
    turningPIDOutput = Math.min(turningPIDOutput, turningPIDMax);

    drive.arcadeDrive(0, turningPIDOutput);
  }

  public void driveWithPID(double driveSetpointLength) {
    drivingPIDController.setSetpoint(driveSetpointLength);
    double drivingPIDOutput = drivingPIDController.calculate(navX.getYaw());

    drivingPIDOutput = Math.max(drivingPIDOutput, turningPIDMin); 
    drivingPIDOutput = Math.min(drivingPIDOutput, turningPIDMax);

    drive.arcadeDrive(0, drivingPIDOutput);
  }

  public boolean isAtSetpoint() {
    return turningPIDController.atSetpoint();
  }

  public boolean isDriveAtSetpoint() {
    return drivingPIDController.atSetpoint();
  }

  public void driveManager() {
    drive.arcadeDrive(
        RobotContainer.m_driverController.getRawAxis(1) * Constants.DriveTrain.kspeedMultiplier * creepSpeed,
        RobotContainer.m_driverController.getRawAxis(4) * Constants.DriveTrain.kspeedMultiplier * creepSpeed);
  }

  public double getSpeed() {
    return leftDrive1.get();
  }

  public void setBrakeMode() {
    leftDrive1.setIdleMode(IdleMode.kBrake);
    leftDrive2.setIdleMode(IdleMode.kBrake);
    rightDrive1.setIdleMode(IdleMode.kBrake);
    rightDrive2.setIdleMode(IdleMode.kBrake);
  }

  public void autoChargeBalance() {
    turningPIDController.setSetpoint(turnSetpointDegree);
    double turningPIDOutput = turningPIDController.calculate(navX.getAngle());

    // Clamping turning PID value between min and max outputs //
    turningPIDOutput = Math.max(turningPIDOutput, turningPIDMin); 
    turningPIDOutput = Math.min(turningPIDOutput, turningPIDMax);

    drive.arcadeDrive(0, turningPIDOutput);
    }

  public double getRollAngle() {
    return navX.getRoll();
  }

  public double getVoltage() {
    return leftDrive1.getBusVoltage();
  }
  }

