// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmSubsystem extends SubsystemBase {

  private TalonSRX armMotor;
  private Encoder armEncoder;
  private double armEncoderDistance;
  private PIDController armPIDController;
  private double m_setpoint;

  
  
  /** Creates a new ArmSystem. */
  public ArmSubsystem() {
    armMotor = new TalonSRX(Constants.Arm.karmMotor);
    armEncoder = new Encoder(2, 1, true, CounterBase.EncodingType.k4X);
    armPIDController = new PIDController(1.6878, 0, 0);
    armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
  }

  @Override
  public void periodic() {
    double outputSpeed = 0;
    if (RobotContainer.m_operatorController.getLeftBumper()) {
      outputSpeed += 0.5;
    }

    if (RobotContainer.m_operatorController.getRightBumper()) {
      outputSpeed -= 0.5;
    }

    armMotorSet(outputSpeed);

    armEncoderDistance = armEncoder.getDistance();

    if (RobotContainer.m_operatorController.getXButtonPressed()) {
      armEncoder.reset();
    }

    armPIDController.setSetpoint(5);

    //armMotor.set(armPIDController.calculate(armEncoder.getDistance(), m_setpoint));

    
    // This method will be called once per scheduler run
  }

  public void armMotorSet(double speed) {
    armMotor.set(ControlMode.PercentOutput, speed);
  }

  public void getArmEncoderValues() {
    System.out.println(armEncoderDistance);
  }

  public void resetArmEncoders() {
    armEncoder.reset();
  }

  public void setSetpoint(double setpoint){
    m_setpoint = setpoint;
  }

  public void armToMidAuto() {
      if(armEncoder.getDistance() > -900) {
        armMotorSet(0.5);
      } else if (armEncoder.getDistance() <= -900) {
        armMotorSet(0);
  }
}
}

