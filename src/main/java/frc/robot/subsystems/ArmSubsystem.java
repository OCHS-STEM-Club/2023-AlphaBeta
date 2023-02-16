// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

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

  private CANSparkMax armMotor;
  private Encoder armEncoder;
  private double armEncoderDistance;
  private SparkMaxPIDController armPIDController;
  private double m_setpoint = 0;
  private double pidReference = 0;


  private SparkMaxLimitSwitch forwardLimitSwitch;
  private SparkMaxLimitSwitch backwardLimitSwitch;
  

  
  
  /** Creates a new ArmSystem. */
  public ArmSubsystem() {
    armMotor = new CANSparkMax(Constants.Arm.karmMotor, MotorType.kBrushless);
    armEncoder = new Encoder(2, 1, true, CounterBase.EncodingType.k4X);
    forwardLimitSwitch = armMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    backwardLimitSwitch = armMotor.getReverseLimitSwitch(Type.kNormallyOpen);


    armPIDController = armMotor.getPIDController();

    armPIDController.setP(10);
    // armPIDController.setI(kI);
    // armPIDController.setD(kD);
    // armPIDController.setIZone(kIz);
    // armPIDController.setFF(kFF);
    armPIDController.setOutputRange(-0.5, 0.5);

    armMotor.setIdleMode(IdleMode.kBrake);

    SmartDashboard.putNumber("PID Set Reference", pidReference);



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

    if (RobotContainer.m_buttonBox.getPOV() == 0) {
      outputSpeed += 0.5;
    }

    if (RobotContainer.m_buttonBox.getPOV() == 180) {
      outputSpeed -= 0.5;
    }

    // armMotorSet(outputSpeed);



    

    armEncoderDistance = armEncoder.getDistance();

    if (backwardLimitSwitch.isPressed()) {
      armEncoder.reset();
    }


    
    if (RobotContainer.m_buttonBox.getYButton()) {
      m_setpoint = Constants.Setpoints.kcarrySetpoint;
    }

    if (RobotContainer.m_buttonBox.getRightBumper()) {
      m_setpoint = Constants.Setpoints.kmidSetpoint;
    }

    if (RobotContainer.m_buttonBox.getLeftBumper()) {
      m_setpoint = Constants.Setpoints.khighSetpoint;
    }

    if (RobotContainer.m_buttonBox.getXButton()) {
      m_setpoint = Constants.Setpoints.kresetSetpoint;
    } 

    // armUpAuto(0.5, m_setpoint);

    // forwardLimitSwitch.enableLimitSwitch(true);

    //System.out.println("limit switch: " + forwardLimitSwitch.isPressed());
    System.out.println("Arm Encoder Value " + armEncoderDistance);

    // armPIDController.g

    pidReference = SmartDashboard.getNumber("PID Set Reference", 0);

    armPIDController.setReference(pidReference, CANSparkMax.ControlType.kPosition);
  

    //armMotor.setNeutralMode(NeutralMode.Brake);

    //armMotor.set(armPIDController.calculate(armEncoder.getDistance(), setpoint));

    //armMotor.set(armPIDController.calculate(armEncoder.getDistance(), m_setpoint));

    
    // This method will be called once per scheduler run

    
  }

  public void armMotorSet(double speed) {
    armMotor.set(speed);
  }


  public void resetArmEncoders() {
    armEncoder.reset();
  }

  public void setSetpoint(double setpoint){
    m_setpoint = setpoint;
  }

  public void armUpAuto(double speed, double setpoint) {
      if(armEncoder.getDistance() > setpoint + 25) { // go up
        armMotorSet(speed);
      } else if (armEncoder.getDistance() < setpoint - 25) { // go down
        armMotorSet(-speed);
      } else {
        armMotorSet(0);
      }
  }

 

}


