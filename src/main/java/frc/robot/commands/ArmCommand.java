// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPositionConstants;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class ArmCommand extends CommandBase 
{
  /** Creates a new ArmCommand. */
  private Shoulder m_shoulder;
  private Wrist m_wrist;
  private String m_position;
  
  private boolean m_isFinished;
  private double m_shoulderSetPoint;
  private double m_wristSetPoint;

  public ArmCommand(Shoulder shoulder, Wrist wrist, String Position)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shoulder = shoulder;
    m_wrist = wrist;
    m_position = Position;

    addRequirements(m_shoulder, m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    m_shoulder.setInitialSetPoint();
    //m_wrist.setInitialSetPoint();
    m_isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
      if (m_position == "STOWED")
      {
        SmartDashboard.putString("Arm Position", "STOWED");
        m_shoulderSetPoint = ArmPositionConstants.shoulderStowed;
        m_wristSetPoint = ArmPositionConstants.wristStowed;
      }
      else if (m_position == "TOPGRIDCONE")
      {
        SmartDashboard.putString("Arm Position", "TOPGRIDCONE");
        m_shoulderSetPoint = ArmPositionConstants.shoulderTopGridCone;
        m_wristSetPoint = ArmPositionConstants.wristTopGridCone;
      }
      else if (m_position == "MIDGRIDCONE")
      {
        SmartDashboard.putString("Arm Position", "MIDGRIDCONE");
        m_shoulderSetPoint = ArmPositionConstants.shoulderMidGridCone;
        m_wristSetPoint = ArmPositionConstants.wristMidGridCone;
      }
      else if (m_position == "TOPGRIDCUBE")
      {
        SmartDashboard.putString("Arm Position", "TOPGRIDCUBE");
        m_shoulderSetPoint = ArmPositionConstants.shoulderTopGridCube;
        m_wristSetPoint = ArmPositionConstants.wristTopGridCube;
      }
      else if (m_position == "MIDGRIDCUBE")
      {
        SmartDashboard.putString("Arm Position", "MIDGRIDCUBE");
        m_shoulderSetPoint = ArmPositionConstants.shoulderMidGridCube;
        m_wristSetPoint = ArmPositionConstants.wristMidGridCube;
      }
      else if (m_position == "FLOOR")
      {
        SmartDashboard.putString("Arm Position", "FLOOR");
        m_shoulderSetPoint = ArmPositionConstants.shoulderFloor;
        m_wristSetPoint = ArmPositionConstants.wristFloor;
      }
      else if (m_position == "SUBSTATION")
      {
        SmartDashboard.putString("Arm Position", "SUBSTATION");
        m_shoulderSetPoint = ArmPositionConstants.shoulderSubstation;
        m_wristSetPoint = ArmPositionConstants.wristSubstation;
      }

    m_shoulder.setSetpoint(m_shoulderSetPoint);
    m_wrist.setSetpoint(m_wristSetPoint);

    //m_isFinished = (m_shoulder.moveEnd() && m_wrist.moveEnd());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_isFinished = true;
    //m_shoulder.forceMotorStop();
    //m_wrist.forceMotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}
