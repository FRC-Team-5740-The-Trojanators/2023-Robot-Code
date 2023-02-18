// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */

  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  public LEDs()
  {

    m_led = new AddressableLED(4);
    m_ledBuffer = new  AddressableLEDBuffer(150);

    m_led.setLength(150);
    m_led.setBitTiming(100, 100, 100, 100);
    for(int i = 0; i<150; i++)
    {
    m_ledBuffer.setRGB(i, 0, 255, 0);
    }
    m_led.setData(m_ledBuffer);

  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    m_led.setData(m_ledBuffer);
  }
}
