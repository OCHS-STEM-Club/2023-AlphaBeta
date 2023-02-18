// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
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

  private double leftEncoderPosition;
  private double rightEncoderPosition;

  private SparkMaxPIDController drivetrainPIDControllerLeft;
  private SparkMaxPIDController drivetrainPIDControllerRight;

  private AutoDriveStraight m_AutoDriveStraight;

 //private final Gyro navX = new AHRS();



  /** Creates a new Drivetrain. */
  public DrivetrainSubsystem() {
    // Set up left drivetrain motors
    leftDrive1 = new CANSparkMax(Constants.DriveTrain.kleftDrive1Id, MotorType.kBrushless);
    leftDrive2 = new CANSparkMax(Constants.DriveTrain.kleftDrive2Id, MotorType.kBrushless);
    leftDrive1.setOpenLoopRampRate(.75);
    leftDrive2.setOpenLoopRampRate(.75);
    

    leftDriveGroup = new MotorControllerGroup(leftDrive1,leftDrive2);
    leftDriveGroup.setInverted(false);

    

    // Set up right drivetrain motors
    rightDrive1 = new CANSparkMax(Constants.DriveTrain.krightDrive1Id, MotorType.kBrushless);
    rightDrive2 = new CANSparkMax (Constants.DriveTrain.krightDrive2Id, MotorType.kBrushless);
    rightDrive1.setOpenLoopRampRate(Constants.DriveTrain.ksetOpenLoopRampRate);
    rightDrive2.setOpenLoopRampRate(Constants.DriveTrain.ksetOpenLoopRampRate);
    rightDriveGroup = new MotorControllerGroup(rightDrive1, rightDrive2);
    rightDriveGroup.setInverted(true);
    
    // Set up Differential Drive
    drive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);

    leftEncoder = leftDrive1.getEncoder();
    rightEncoder = rightDrive1.getEncoder();
    
    // compute conversion from encoder units to meters
    double conversionFactor = ((1 / Constants.DriveTrain.kdriveEncoderNativeUnitsPerRev) / Constants.DriveTrain.kdriveGearRatio) * 
    (Math.PI * Constants.DriveTrain.kwheelDiameterInches) * Constants.DriveTrain.kinchesToMeters;

    // Set conversion factor for both encoders
    leftEncoder.setPositionConversionFactor(conversionFactor);
    rightEncoder.setPositionConversionFactor(conversionFactor);

    leftEncoderPosition = leftEncoder.getPosition();
    rightEncoderPosition = rightEncoder.getPosition();

    drivetrainPIDControllerLeft = leftDrive1.getPIDController();
    drivetrainPIDControllerRight = rightDrive1.getPIDController();

    drivetrainPIDControllerLeft.setP(0.047617);
    drivetrainPIDControllerRight.setP(0.047617);

    }

    

  @Override
  public void periodic() {
  
    

    //double xStickValue = RobotContainer.m_driverController.getRawAxis(4) * Constants.DriveTrain.kspeedMultiplier * creepSpeed;
    //double yStickValue = -RobotContainer.m_driverController.getRawAxis(1) * Constants.DriveTrain.kspeedMultiplier * creepSpeed;
    // This method will be called once per scheduler run
  }
  
  public void driveWithXbox() {
    if(RobotContainer.m_driverController.getRawAxis(2) == 1){
      creepSpeed = 0.5;
  }else if(RobotContainer.m_driverController.getRawAxis(3) == 1){
      creepSpeed = 1;
  }else {
      creepSpeed = 1;
  }
    //System.out.printf("Controller: forward: %f, turn: %f\n", xboxController.getLeftY(), xboxController.getRightX());
    drive.arcadeDrive(
      RobotContainer.m_driverController.getRawAxis(1) * Constants.DriveTrain.kspeedMultiplier * creepSpeed, 
      RobotContainer.m_driverController.getRawAxis(4) * Constants.DriveTrain.kspeedMultiplier * creepSpeed
    );

    System.out.println("Left Encoder Distance " + leftEncoder.getPosition());
    
    // drive.arcadeDrive(1, 0);
  }

  

  public void resetEncoders() {
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);

  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public void auto(double distance, double speed){
		if(leftEncoder.getPosition() > distance){
			drive.arcadeDrive(-speed, 0.0);
		} else if (leftEncoder.getPosition() <= distance) {
      drive.arcadeDrive(0.0, 0.0);
      m_AutoDriveStraight.end(true);
    } 
     

	}

  public void setEncodersToZero() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

}
