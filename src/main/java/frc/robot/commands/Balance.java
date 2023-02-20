// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Balance extends CommandBase {
  DriveSubsystem m_drivetrain;  
  boolean m_isFinished;
  boolean m_isX;

  /** Creates a new Xbalance. */
  public Balance(DriveSubsystem drivetrain, boolean isX) 
  {
    m_drivetrain = drivetrain;
    m_isX = isX;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(m_isX)
    {
      if(m_drivetrain.getPitch(true).getDegrees() < -SwerveDriveModuleConstants.k_balanceDeadband)
      {
        m_drivetrain.teleDrive(0, 1, 0, false);
      } 
      else
      if(m_drivetrain.getPitch(true).getDegrees() > SwerveDriveModuleConstants.k_balanceDeadband)
        {
          m_drivetrain.teleDrive(0, -1, 0, false);
        } 
        else 
        {
          m_drivetrain.teleDrive(0, 0, 0, false);
        }
    }
    else 
    {
      if(m_drivetrain.getRoll(true).getDegrees() < -SwerveDriveModuleConstants.k_balanceDeadband)
      {
        m_drivetrain.teleDrive(1, 0, 0, false);
      } 
      else
      if(m_drivetrain.getRoll(true).getDegrees() > SwerveDriveModuleConstants.k_balanceDeadband)
        {
          m_drivetrain.teleDrive(-1, 0, 0, false);
        } 
        else 
        {
          m_drivetrain.teleDrive(0, 0, 0, false);
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
