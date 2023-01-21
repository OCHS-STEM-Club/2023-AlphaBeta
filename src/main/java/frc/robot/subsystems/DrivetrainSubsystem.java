// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

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

 private final Gyro navX = new AHRS();


private final DifferentialDriveOdometry m_odometry;



  /** Creates a new Drivetrain. */
  public DrivetrainSubsystem() {
    // Set up left drivetrain motors
    leftDrive1 = new CANSparkMax(Constants.DriveTrain.kleftDrive1Id, MotorType.kBrushless);
    leftDrive2 = new CANSparkMax(Constants.DriveTrain.kleftDrive2Id, MotorType.kBrushless);
    leftDriveGroup = new MotorControllerGroup(leftDrive1, leftDrive2);
    leftDriveGroup.setInverted(true);

    // Set up right drivetrain motors
    rightDrive1 = new CANSparkMax(Constants.DriveTrain.krightDrive1Id, MotorType.kBrushless);
    rightDrive2 = new CANSparkMax (Constants.DriveTrain.krightDrive2Id, MotorType.kBrushless);
    rightDriveGroup = new MotorControllerGroup(rightDrive1, rightDrive2);
    rightDriveGroup.setInverted(false);
    
    // Set up Differential Drive
    drive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);

    leftEncoder = leftDrive1.getEncoder();
    rightEncoder = rightDrive1.getEncoder();
    
    // compute conversion from encoder units to meters
    double conversionFactor = (((1 / Constants.DriveTrain.kdriveEncoderNativeUnitsPerRev) / Constants.DriveTrain.kdriveGearRatio) * 
    Math.PI * Constants.DriveTrain.kwheelDiameterInches) * Constants.DriveTrain.kinchesToMeters;

    // Set conversion factor for both encoders
    leftEncoder.setPositionConversionFactor(conversionFactor);
    rightEncoder.setPositionConversionFactor(conversionFactor);

    // Resetting all parameters to 0
    resetOdometryParameters();
    m_odometry =
    new DifferentialDriveOdometry(
      navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void driveWithXbox(XboxController xboxController) {
    // System.out.printf("Controller: forward: %f, turn: %f\n", xboxController.getLeftY(), xboxController.getRightX());
    drive.arcadeDrive(
      xboxController.getLeftY()*Constants.DriveTrain.kspeedMultiplier, 
      xboxController.getRightX()*Constants.DriveTrain.kspeedMultiplier
    );
    
    // drive.arcadeDrive(1, 0);
  }

  public void testMotors() {
    // leftDriveGroup.setVoltage(7);
    // rightDriveGroup.setVoltage(7);
    leftDriveGroup.set(1);
  }

  public void mobilitySubsystem() {
    drive.arcadeDrive(-0.5, 0);
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters(); 
  }

  public void resetOdometryParameters() {
    leftEncoder.setPosition(0); 
    rightEncoder.setPosition(0);
    navX.reset();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDriveGroup.setVoltage(leftVolts);
    rightDriveGroup.setVoltage(rightVolts);
      drive.feed();
  }

  public void resetOdometry(Pose2d pose) {
    resetOdometryParameters();
    m_odometry.resetPosition(
      navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
  }
}
