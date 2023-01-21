// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
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

 //private final Gyro navX = new AHRS();






  /** Creates a new Drivetrain. */
  public DrivetrainSubsystem() {
    // Set up left drivetrain motors
    leftDrive1 = new CANSparkMax(Constants.DriveTrain.kleftDrive1Id, MotorType.kBrushless);
    leftDrive2 = new CANSparkMax(Constants.DriveTrain.kleftDrive2Id, MotorType.kBrushless);
    leftDriveGroup = new MotorControllerGroup(leftDrive1, leftDrive2);
    leftDriveGroup.setInverted(false);

    // Set up right drivetrain motors
    rightDrive1 = new CANSparkMax(Constants.DriveTrain.krightDrive1Id, MotorType.kBrushless);
    rightDrive2 = new CANSparkMax (Constants.DriveTrain.krightDrive2Id, MotorType.kBrushless);
    rightDriveGroup = new MotorControllerGroup(rightDrive1, rightDrive2);
    rightDriveGroup.setInverted(true);
    
    // Set up Differential Drive
    drive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);


    /* 
    leftEncoder = leftDrive1.getEncoder();
    rightEncoder = rightDrive1.getEncoder();
    
    // compute conversion from encoder units to meters
    double conversionFactor = (((1 / Constants.DriveTrain.kdriveEncoderNativeUnitsPerRev) / Constants.DriveTrain.kdriveGearRatio) * 
    Math.PI * Constants.DriveTrain.kwheelDiameterInches) * Constants.DriveTrain.kinchesToMeters;

    // Set conversion factor for both encoders
    leftEncoder.setPositionConversionFactor(conversionFactor);
    rightEncoder.setPositionConversionFactor(conversionFactor);
*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void driveWithXbox(CommandXboxController xboxController) {
    //System.out.printf("Controller: forward: %f, turn: %f\n", xboxController.getLeftY(), xboxController.getRightX());
    drive.arcadeDrive(
      xboxController.getLeftY()*Constants.DriveTrain.kspeedMultiplier, 
      xboxController.getRightX()*Constants.DriveTrain.kspeedMultiplier
    );
    
    // drive.arcadeDrive(1, 0);
  }

  // public void testMotors() {
  //   // leftDriveGroup.setVoltage(7);
  //   // rightDriveGroup.setVoltage(7);
  //   leftDriveGroup.set(1);
  // }
}
