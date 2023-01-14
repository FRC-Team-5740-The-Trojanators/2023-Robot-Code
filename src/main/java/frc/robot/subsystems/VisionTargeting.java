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
  /** Creates a new VisionTargeting. */
  public VisionTargeting() {}

  @Override
  public void periodic() {
         //limelight code to be put onto shuffleboard
NetworkTableEntry limelight = NetworkTableInstance.getDefault().getTable("").getEntry("limelight");
NetworkTable table;
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

//read values periodically
double x = tx.getDouble(0.0);
double y = ty.getDouble(0.0);
double area = ta.getDouble(0.0);

//post to smart dashboard periodically
SmartDashboard.putNumber("Limelight Y", tx);
SmartDashboard.putNumber("Limelight Y", ty);
SmartDashboard.putNumber("LimelightArea", ta);

m_ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
m_txRad = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0) * (Math.PI/180);
}

public double getAimPID()
{
return m_aimPID.calculate(m_txRad, 0);
}

public double getLimelightTY()
{
return m_ty;
}

public boolean seesTarget() 
{
double seesTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
if (seesTarget == 1.0) 
{
return true;
} 
else 
{
return false;
}
}

public double getHeadingToTarget() 
{
double seesTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
if (seesTarget == 1.0) 
{
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)
        + NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
} 
else 
{
    return 0.0;
}
}

public boolean aimEnd()
{
    return m_aimPID.atSetpoint();
}

public double getSkew() 
{
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
}

public double getHeight()
{
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
}

public double getX()
{
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
}

public double getY()
{
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
}

public void ledOn()
{
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
}

public void ledOff()
{
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
}
    // This method will be called once per scheduler run
  }
}
