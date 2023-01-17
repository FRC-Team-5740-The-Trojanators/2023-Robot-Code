// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionTargeting extends SubsystemBase {
    private double m_tx;
    private double m_tv;

/** Creates a new VisionTargeting. */
  public VisionTargeting() {

  }

  @Override
  public void periodic() {
         //limelight code to be put onto shuffleboard
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry tv = table.getEntry("tv");
   
    //read values periodically
    double x = tx.getDouble(0.0);
    double v = tv.getDouble(0.0);
 
    
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightV", v);

}

/*public double getAimPID()
{
return m_aimPID.calculate(m_txRad, 0);
}
*/

public double getLimelightTX()
{
return m_tx;
}

public boolean seesTarget() 
{;
if (m_tv == 1.0) 
{
return true;
} 
else 
{
return false;
}
}

public double getSkew() 
{
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
}

public double getHeadingToTarget() 
{
if (seesTarget()) 
{
    return m_tx + getSkew();
} 
else 
{
    return 0.0;
}
}

//public boolean aimEnd()
//{
//    return m_aimPID.atSetpoint();
//}


public void ledOn()
{
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
}

public void ledOff()
{
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
}
  
}
