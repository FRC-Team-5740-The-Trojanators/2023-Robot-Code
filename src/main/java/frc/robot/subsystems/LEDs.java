


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.LEDColor;


/*

public class LEDs extends SubsystemBase 
{
  /** Creates a new LEDs. 
  LEDColor m_red; 
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;

  public class LEDColor 
  {
    private int r;
    private int g;
    private int b;

    public LEDColor(int r, int g, int b)
    {
        this.r = r;
        this.g = g;
        this.b = b;
        m_red = new LEDColor(255, 0, 0);
    }

    public int getR(){
        return r;
    }
    public int getG(){
        return g;
    }
    public int getB(){
        return b;
    }

    
    //public static final LEDColor m_red = new LEDColor(255, 0, 0);
    public static final LEDColor kGreen = new LEDColor(0, 255, 0);
    public static final LEDColor kBlue = new LEDColor(0, 0, 255);
    public static final LEDColor kYellow = new LEDColor(255, 255, 0);
    public static final LEDColor kPurple = new LEDColor(127, 0, 255);
    public static final LEDColor kOrange = new LEDColor(255, 95, 31);

    //public static final LEDColor kOff = new LEDColor(0, 0, 0);
    }


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

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run\
    //m_led.setData(m_ledBuffer);
    //m_led.start();


  }
}




*/