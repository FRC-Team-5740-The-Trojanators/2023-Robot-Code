// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeAndTaxi extends SequentialCommandGroup {
  /** Creates a new ScoreConeAndTaxi. */
  private DriveSubsystem m_driveSubsystem;
  private Claw m_claw;
  private Shoulder m_shoulder;
  private Wrist m_wrist;

  public ScoreConeAndTaxi(DriveSubsystem driveSubsystem, Claw claw, Shoulder shoulder, Wrist wrist)
  {
    m_driveSubsystem = driveSubsystem;
    m_claw = claw;
    m_shoulder = shoulder;
    m_wrist = wrist;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem, m_claw, m_shoulder, m_wrist);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    ArrayList<PathPlannerTrajectory> pathGroup1 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("ScoreConeAndTaxi", new PathConstraints(1.5, 1));

    addCommands(
      new ParallelDeadlineGroup(
            new WaitCommand(2),
            new RunClawCommand(m_claw, "FORWARD"),
            new ArmCommand(m_shoulder, m_wrist, "TOPGRIDCONE")),
      new InstantCommand(() -> {
                  m_driveSubsystem.resetOdometry(pathGroup1.get(0).getInitialHolonomicPose());
            }),
      new ParallelDeadlineGroup(
            new PPSwerveControllerCommand(
                  pathGroup1.get(0),
                  m_driveSubsystem::getPose,
                  SwerveDriveModuleConstants.k_AutoKinematics, // SwerveDriveKinematics
                  new PIDController(SwerveDriveModuleConstants.k_pTransController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                  new PIDController(SwerveDriveModuleConstants.k_pTransController, 0.0, 0.0),
                  new PIDController(SwerveDriveModuleConstants.k_pThetaController, 0.0, 0.0),
                  m_driveSubsystem::setSwerveModuleStatesAuto,
                  false,
                  m_driveSubsystem),
            new RunClawCommand(m_claw, "FORWARD")),
      new WaitCommand(.2),
      new ParallelDeadlineGroup(
            new WaitCommand(.5),
            new RunClawCommand(m_claw, "BACKWARD")),
      new ParallelDeadlineGroup(
            new PPSwerveControllerCommand(
                pathGroup1.get(1),
                m_driveSubsystem::getPose,
                SwerveDriveModuleConstants.k_AutoKinematics, // SwerveDriveKinematics
                new PIDController(SwerveDriveModuleConstants.k_pTransController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                new PIDController(SwerveDriveModuleConstants.k_pTransController, 0.0, 0.0),
                new PIDController(SwerveDriveModuleConstants.k_pThetaController, 0.0, 0.0),
                m_driveSubsystem::setSwerveModuleStatesAuto,
                false,
                m_driveSubsystem),
            new RunClawCommand(m_claw, "BACKWARD")),
      new ArmCommand(m_shoulder, m_wrist, "STOWED")
    );
  }
}
