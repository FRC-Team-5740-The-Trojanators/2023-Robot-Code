


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SetColor.LEDColor;


public class LEDs extends SubsystemBase 
{
  //Creates a new LEDs. 
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  private boolean isOn;
  private double lastChange; 

  public LEDs()
  {    
    m_led = new AddressableLED(4);
   
    m_ledBuffer = new  AddressableLEDBuffer(300);
    
    m_led.setLength(m_ledBuffer.getLength());
    //m_led.setBitTiming(0, 1, 0, 1);

    for(int i = 0; i<m_ledBuffer.getLength(); i++)
    {
    m_ledBuffer.setRGB(i, 0, 0, 255);
    }

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setColor(LEDColor color)
  {
    for(int i = 0; i < m_ledBuffer.getLength(); i++){
      m_ledBuffer.setRGB(i, color.getR(), color.getG(), color.getB()); 
    }
    m_led.setData(m_ledBuffer);
  }

  public void pulse(LEDColor color, double interval)
  {
    double timestamp = Timer.getFPGATimestamp();
    if(timestamp -lastChange > interval)
    {
      lastChange = timestamp;
      isOn = !isOn;
    }
    if(isOn)
    {
      stop();
    }
    else
    {
      setColor(color);
    }
  }

  public void stop()
  {
    setColor(Color.kOff);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run\
  }
}




