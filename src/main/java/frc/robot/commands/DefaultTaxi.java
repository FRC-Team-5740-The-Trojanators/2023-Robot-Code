// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import lib.swerve.SwervePath;
import lib.swerve.SwervePathController;

public class DefaultTaxi extends CommandBase {

  private final DriveSubsystem driveSubsystem;

  private Timer timer;
  SwervePath path;
  SwervePathController pathController;
  double lastTime;
  boolean ignoreHeading;
  /** Creates a new DefaultTaxi. */
  public DefaultTaxi(DriveSubsystem driveSubsystem) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.path = SwervePath.fromCSV("Default taxi");   
    this.timer = new Timer();
    this.ignoreHeading = false;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer = new Timer();
    timer.reset();
    timer.start(); 
    SwervePath.State initialState = path.getInitialState();

    //driveSubsystem.resetOdometry(new Pose2d(driveSubsystem.getPoseMeters().getTranslation(), initialState.getRotation()));
    driveSubsystem.resetEncoders();
    
    lastTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double time = timer.get();
    SwervePath.State desiredState = path.sample(time);

    if(ignoreHeading) desiredState.rotation = new Rotation2d(0);

    //driveSubsystem.autoDrive(desiredState.getVelocity() * desiredState.getRotation().getCos(), desiredState.getVelocity() * desiredState.getRotation().getSin(), 0, false);

    lastTime = time;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    timer.stop();
    //driveSubsystem.autoDrive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return timer.hasElapsed(path.getRuntime());
  }
}
