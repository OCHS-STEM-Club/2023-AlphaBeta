// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void driveWithXbox(CommandXboxController xboxController) {
    drive.arcadeDrive(
      xboxController.getLeftY()*Constants.DriveTrain.kspeedMultiplier, 
      xboxController.getRightX()*Constants.DriveTrain.kspeedMultiplier
    );
  }
}