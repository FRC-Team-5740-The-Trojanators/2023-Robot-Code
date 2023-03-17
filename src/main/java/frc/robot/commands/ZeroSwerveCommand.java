// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ZeroSwerveCommand extends CommandBase {
  /** Creates a new ZeroSwerveCommand. */
  private final DriveSubsystem drivetrain;
  private boolean m_isfinished;
  
  public ZeroSwerveCommand(DriveSubsystem drivetrain) 
  {
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_isfinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    drivetrain.setZeroState();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_isfinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isfinished;
  }
}
