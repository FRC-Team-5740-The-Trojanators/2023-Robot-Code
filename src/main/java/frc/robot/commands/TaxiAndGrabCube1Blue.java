// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
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
public class TaxiAndGrabCube1Blue extends SequentialCommandGroup
{
  /** Creates a new TaxiAndGrabCube1Blue. */
  private DriveSubsystem m_driveSubsystem;
  private Claw m_claw;
  private Shoulder m_shoulder;
  private Wrist m_wrist;
  private HashMap<String, Command> m_eventMap;
  
  public TaxiAndGrabCube1Blue(DriveSubsystem driveSubsystem, Claw claw, Shoulder shoulder, Wrist wrist,  HashMap<String, Command> eventMap) 
  {
    
    m_driveSubsystem = driveSubsystem;
    m_claw = claw;
    m_shoulder = shoulder;
    m_wrist = wrist;
    m_eventMap = eventMap;

    addRequirements(m_driveSubsystem, m_claw, m_shoulder, m_wrist);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory path = PathPlanner.loadPath("TaxiAndGrabCube1Blue", new PathConstraints(2, 2));
    //ArrayList<PathPlannerTrajectory> pathGroup1 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("TaxiAndGrabCube1Blue", new PathConstraints(3, 2));
    addCommands
    (
      new FollowPathWithEvents(
        new PPSwerveControllerCommand(
          path,
          m_driveSubsystem::getPose,
          SwerveDriveModuleConstants.k_AutoKinematics, // SwerveDriveKinematics
          new PIDController(SwerveDriveModuleConstants.k_pTransController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
          new PIDController(SwerveDriveModuleConstants.k_pTransController, 0.0, 0.0),
          new PIDController(SwerveDriveModuleConstants.k_pThetaController, 0.0, 0.0),
          m_driveSubsystem::setSwerveModuleStatesAuto,
          false,
          m_driveSubsystem),
        path.getMarkers(),
        m_eventMap
      ),
      new ParallelDeadlineGroup(new WaitCommand(.5), new RunClawCommand(m_claw, "BACKWARD"))
    );
  }
}
