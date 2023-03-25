// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Balance extends CommandBase {
  DriveSubsystem m_drivetrain;  
  boolean m_isFinished;
  double angleDegrees;
  double positionThresholdDegrees = 3.0;
  double velocityThresholdDegreesPerSec = 3.0;
  double speedInchesPerSec = 22.5;

  /** Creates a new Xbalance. */
  public Balance(DriveSubsystem drivetrain) 
  {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    angleDegrees = m_drivetrain.getPitch(false).getDegrees();
    
    double angleVelocityDegreesPerSec = m_drivetrain.getPitchVelocity();

    boolean shouldStop = Math.abs(angleVelocityDegreesPerSec) > velocityThresholdDegreesPerSec;
            //|| (angleDegrees > 0.0 && Math.abs(angleVelocityDegreesPerSec) < -velocityThresholdDegreesPerSec);

    // Send velocity to drive
    if (shouldStop) {
      m_drivetrain.teleDrive(0, 0, 0, true);
    } else {
      m_drivetrain.teleDrive(
              Units.inchesToMeters(speedInchesPerSec) * (angleDegrees < 0.0 ? -1.0 : 1.0),
              0.0,
              0.0,
              true);
    }

    SmartDashboard.putBoolean("Should Stop", shouldStop);
    SmartDashboard.putNumber("Angle Degrees", angleDegrees);
    //SmartDashboard.putNumber("Angular Velocity", angleVelocityDegreesPerSec);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_drivetrain.teleDrive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    //return Math.abs(angleDegrees) < positionThresholdDegrees;
    return false;
  }
}
