// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
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
  }

  public int getR()
  {
    SmartDashboard.putString("RED", "SHOWING");
      return r;
  }
  public int getG()
  {
    SmartDashboard.putString("GREEN", "SHOWING");
      return g;
  }
  public int getB()
  {
    SmartDashboard.putString("BLUE", "SHOWING");
      return b;
  }
}
