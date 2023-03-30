// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */

  private AddressableLED m_led = new AddressableLED(Constants.LED.PWPORT);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LED.BUFFERSIZE);
  private float hue = 0;
  DriverStation.Alliance color;

  public LEDSubsystem() {
    System.out.println("LED Initalized");
    initializeLEDLength();

    color = DriverStation.getAlliance();
        if (color == DriverStation.Alliance.Blue) {
          setFrontAll(Color.kBlue);
        } else if (color == DriverStation.Alliance.Red) {
          setFrontAll(Color.kRed);
        } else setFrontAll(Color.kBlue);
    
  }

  @Override
  public void periodic() {
    //setFrontAll(Color.kWhite);
    setBallLEDs();
    // This method will be called once per scheduler run
  }

  public void setFrontAll(Color color) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        m_ledBuffer.setLED(i, color);
      } else m_ledBuffer.setLED(i, Color.kBlack);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setFrontAllRGB(int r, int g, int b) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      if (i % 2 == 0) {
        m_ledBuffer.setRGB(i, r, g, b);
      } else m_ledBuffer.setLED(i, Color.kBlack);
    }
    m_led.setData(m_ledBuffer);
  }

    public void setBallLEDs() {
      if(RobotContainer.m_buttonBox.getRightTriggerAxis() > 0) {
        setFrontAllRGB(255, 180, 0);
      } 
      if(RobotContainer.m_buttonBox.getLeftTriggerAxis() > 0) {
        setFrontAll(Color.kPurple);
      }

      if(RobotContainer.m_buttonBox.getRawButton(9)) {
        setFrontAll(Color.kBlack);
      }


      if(RobotContainer.m_buttonBox.getRawButton(10)) {
        color = DriverStation.getAlliance();
        if (color == DriverStation.Alliance.Blue) {
          setFrontAll(Color.kBlue);
        } else setFrontAll(Color.kRed);
      }

      // else {
      //   setFrontAll(Color.kBlue);
      // }

      //setFrontAll(color);
    }

    public void initializeLEDLength() {
      m_led.setLength(m_ledBuffer.getLength());
      m_led.setData(m_ledBuffer);
      m_led.start();
    }

    public void setColor(Color color) {
      setFrontAll(color);
    }
  
}
