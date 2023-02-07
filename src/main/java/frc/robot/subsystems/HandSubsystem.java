// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class HandSubsystem extends SubsystemBase {

  private TalonSRX handMotorLeft;
  private TalonSRX handMotorRight;
  private MotorControllerGroup handMotor;
  private Ultrasonic ultrasonicSensor;
  private double ultrasonicSensorDistanceMM; 
  private double ultrasonicSensorDistanceIn;
  
  /** Creates a new HandSubsystem. */
  public HandSubsystem() {
    handMotorLeft = new TalonSRX(9); //TODO: Change ID//
    handMotorRight = new TalonSRX(10); //TODO: Change ID//\
    ultrasonicSensor = new Ultrasonic(4, 5);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double outputSpeed = 0;
    if (RobotContainer.m_driverController.getRightBumper()) outputSpeed += 0.5;
    
    if (RobotContainer.m_driverController.getLeftBumper()) outputSpeed -= 0.5;

    spinHandMotors(outputSpeed);
    
   
    ultrasonicSensorDistanceMM = ultrasonicSensor.getRangeMM();
    ultrasonicSensorDistanceIn = ultrasonicSensor.getRangeInches();

  
    //SmartDashboard.putNumber("ultrasonicSensorDistance",ultrasonicSensorDistance);

  }

  public void spinHandMotors(double speed) {
    handMotorLeft.set(ControlMode.PercentOutput, -speed);
    handMotorRight.set(ControlMode.PercentOutput, speed);
    //System.out.println(speed);
  }

//    public void getUltrasonicSensorDistance() {
//      SmartDashboard.putNumber("ultrasonicSensorDistance",ultrasonicSensorDistance);
//  }

  public void getUltrasonicSensorDistance() {
   System.out.println(ultrasonicSensorDistanceMM);
   System.out.println(ultrasonicSensorDistanceIn);
  }

  public void setAutomaticMode() {
  Ultrasonic.setAutomaticMode(true);
  }
}
