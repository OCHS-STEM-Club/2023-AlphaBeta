// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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

  public LEDSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFrontAll(Color color) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    
    }
    m_led.setData(m_ledBuffer);
  }

    public void setBallLEDs() {
      if(RobotContainer.m_buttonBox.getRightTriggerAxis() > 0) {
        setFrontAll(Color.kYellow);
      } else if(RobotContainer.m_buttonBox.getLeftTriggerAxis() > 0) {
        setFrontAll(Color.kDarkOrchid);
      }else {
        setFrontAll(Color.kBlue);
      }
    }

    public void initializeLEDLength() {
      m_led.setLength(m_ledBuffer.getLength());
      m_led.setData(m_ledBuffer);
      m_led.start();
    }
  
}
