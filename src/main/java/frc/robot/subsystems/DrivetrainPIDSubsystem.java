// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class DrivetrainPIDSubsystem extends PIDSubsystem {

  private CANSparkMax leftDrive1;
  private CANSparkMax leftDrive2;
  private MotorControllerGroup leftDriveGroup;
  private CANSparkMax rightDrive1;
  private CANSparkMax rightDrive2;
  private MotorControllerGroup rightDriveGroup;
  private DifferentialDrive drive;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private double leftEncoderPosition;
  private double rightEncoderPosition;
  /** Creates a new DrivetrainPIDSubsystem. */
  public DrivetrainPIDSubsystem() {
    super(new PIDController(0, 0, 0));
        // The PIDController used by the subsystem
       
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
    double conversionFactor = (((1 / Constants.DriveTrain.kdriveEncoderNativeUnitsPerRev) / Constants.DriveTrain.kdriveGearRatio) * 
    Math.PI * Constants.DriveTrain.kwheelDiameterInches) * Constants.DriveTrain.kinchesToMeters;

    // Set conversion factor for both encoders
    leftEncoder.setPositionConversionFactor(conversionFactor);
    rightEncoder.setPositionConversionFactor(conversionFactor);

    leftEncoderPosition = leftEncoder.getPosition();
    rightEncoderPosition = rightEncoder.getPosition();

    
  }

  @Override
  public void useOutput(double output, double setpoint) {
    leftDriveGroup.set(output + getController().calculate(getMeasurement(), setpoint));
    rightDriveGroup.set(output + getController().calculate(getMeasurement(), setpoint));
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return leftEncoder.getPosition();
  }
}
