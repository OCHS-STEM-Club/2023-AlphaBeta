// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  private TalonFX armMotor;
  /** Creates a new ArmSystem. */
  public ArmSubsystem() {
    armMotor = new TalonFX(Constants.Arm.karmMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void armMotorUp(XboxController xboxController) {
    armMotor.set(ControlMode.PercentOutput, xboxController.getPOV(0) * Constants.DriveTrain.kspeedMultiplier );
  }

  public void armMotorDown(XboxController xboxController) {
    armMotor.set(ControlMode.PercentOutput, xboxController.getPOV(180) * Constants.DriveTrain.kspeedMultiplier );
  }
}
