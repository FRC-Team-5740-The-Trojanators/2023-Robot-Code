// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HIDConstants;
import frc.robot.Constants.LEDsSubsystemConstants;
import frc.robot.Constants.SwerveDriveModuleConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.Balance;
import frc.robot.commands.RunClawCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TargetCommand;
import frc.robot.commands.ZeroSwerveCommand;
import frc.robot.commands.SetColor;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.VisionTargeting;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem(false);
  public final VisionTargeting m_visionTargeting = new VisionTargeting();
  private final Claw m_claw = new Claw();
  private final Shoulder m_shoulder = new Shoulder();
  private final Wrist m_wrist = new Wrist();

  //private final RunClawCommand m_runClawCommand = new RunClawCommand(m_claw, "stop");

  public final static LEDs m_leds = new LEDs(LEDsSubsystemConstants.k_port, LEDsSubsystemConstants.k_numLeds);
  XboxController m_driverController = new XboxController(HIDConstants.k_DriverControllerPort);
  XboxController m_operatorController = new XboxController(HIDConstants.k_OperatorControllerPort);

   // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
     //List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(1, 1));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
     HashMap<String, Command> eventMap = new HashMap<>();
  //eventMap.put("marker1", new PrintCommand("Passed marker 1"));
   //eventMap.put("intakeDown", new IntakeDown());

// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      m_driveSubsystem::getPose, // Pose2d supplier
      m_driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
      SwerveDriveModuleConstants.k_AutoKinematics, // SwerveDriveKinematics
      new PIDConstants(SwerveDriveModuleConstants.k_pTransController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(SwerveDriveModuleConstants.k_pThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      m_driveSubsystem::setSwerveModuleStatesAuto, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      m_driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
    );
  
  //private final DefaultTaxi m_autoDefault = new DefaultTaxi(m_driveSubsystem);
  private final SwerveDriveCommand m_driveCommand = new SwerveDriveCommand(m_driveSubsystem, m_driverController);
  
  public static JoystickButton coneTarget, cubeTarget, xBalance, yBalance, aprilTag, zeroDrive, purpleLED, offLED, yellowLED, tapeTarget, runClaw, reverseClaw, stowArm, topGridArm, midGridArm, floorArm, clawIn, clawOut;

  SendableChooser<CommandBase> auto = new SendableChooser<CommandBase>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    configureButtonBindings();

    m_driveSubsystem.setDefaultCommand(m_driveCommand);
    m_shoulder.setDefaultCommand(new ArmCommand(m_shoulder, m_wrist, "STOWED"));
    m_wrist.setDefaultCommand(new ArmCommand(m_shoulder, m_wrist, "STOWED"));
    m_driveSubsystem.resetIMU();
  
    configChooser();
  
  }

  private void configChooser()
  {
    auto.addOption("Position1Cone2Chrg", new SequentialCommandGroup(new ZeroSwerveCommand(m_driveSubsystem),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position1Part1Pickup1", new PathConstraints(4, 3))),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position1Part2Cone", new PathConstraints(4, 3))), autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position1Part3ConeBalance", new PathConstraints(4, 3))) ));
    auto.addOption("Position1Cube2Chrg", new SequentialCommandGroup(new ZeroSwerveCommand(m_driveSubsystem),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position1Part1Pickup1", new PathConstraints(4, 3))), autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position1Part2Cube", new PathConstraints(4, 3))),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position1Part3CubeBalance", new PathConstraints(4, 3))) ));
    auto.addOption("Position1Cone3PckUp", new SequentialCommandGroup(new ZeroSwerveCommand(m_driveSubsystem),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position1Part1Pickup1", new PathConstraints(4, 3))),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position1Part2Cone", new PathConstraints(4, 3))), autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position1Part3ConePickup2", new PathConstraints(4, 3))) ));
    auto.addOption("Position1Cube3PckUp", new SequentialCommandGroup(new ZeroSwerveCommand(m_driveSubsystem),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position1Part1Pickup1", new PathConstraints(4, 3))), autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position1Part2Cube", new PathConstraints(4, 3))),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position1Part3CubePickup2", new PathConstraints(4, 3))) ));
  
    auto.addOption("Position2Cone1Chrg", new SequentialCommandGroup(new ZeroSwerveCommand(m_driveSubsystem),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position2Part1Cone", new PathConstraints(4, 3)))));
    auto.addOption("Position2Cube1Chrg", new SequentialCommandGroup(new ZeroSwerveCommand(m_driveSubsystem),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position2Part1Cube", new PathConstraints(4, 3)))));

    auto.addOption("Position3Cone2Chrg", new SequentialCommandGroup(new ZeroSwerveCommand(m_driveSubsystem),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position3Part1Pickup4", new PathConstraints(4, 3))),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position3Part2Cone", new PathConstraints(4, 3))), autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position3Part3ConeBalance", new PathConstraints(4, 3))) ));
    auto.addOption("Position3Cube2Chrg", new SequentialCommandGroup(new ZeroSwerveCommand(m_driveSubsystem),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position3Part1Pickup4", new PathConstraints(4, 3))), autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position3Part2Cube", new PathConstraints(4, 3))),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position3Part3CubeBalance", new PathConstraints(4, 3))) ));
    auto.addOption("Position3Cone3PckUp", new SequentialCommandGroup(new ZeroSwerveCommand(m_driveSubsystem),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position3Part1Pickup4", new PathConstraints(4, 3))),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position3Part2Cone", new PathConstraints(4, 3))), autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position3Part3ConePickup3", new PathConstraints(4, 3))) ));
    auto.addOption("Position3Cube3PckUp", new SequentialCommandGroup(new ZeroSwerveCommand(m_driveSubsystem),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position3Part1Pickup4", new PathConstraints(4, 3))), autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position3Part2Cube", new PathConstraints(4, 3))),autoBuilder.fullAuto( PathPlanner.loadPathGroup("Position3Part3CubePickup3", new PathConstraints(4, 3))) ));

    auto.addOption("DefaultTaxi", new SequentialCommandGroup(new ZeroSwerveCommand(m_driveSubsystem),autoBuilder.fullAuto( PathPlanner.loadPathGroup("DefaultTaxi", new PathConstraints(4, 3)))));

    SmartDashboard.putData(auto);
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    coneTarget = new JoystickButton(m_driverController , HIDConstants.kA);
    cubeTarget = new JoystickButton(m_driverController , HIDConstants.kB);
    tapeTarget = new JoystickButton(m_driverController, HIDConstants.kX);
    xBalance = new JoystickButton(m_driverController, HIDConstants.kX);
    yBalance = new JoystickButton(m_driverController, HIDConstants.kY);
    aprilTag = new JoystickButton(m_driverController , HIDConstants.kBack);
    zeroDrive = new JoystickButton(m_driverController, HIDConstants.kStart);

    //runClaw = new JoystickButton(m_operatorController, HIDConstants.kA);
    //reverseClaw = new JoystickButton(m_operatorController, HIDConstants.kB); 

    //purpleLED = new JoystickButton(m_operatorController, HIDConstants.kLB);
    //yellowLED = new JoystickButton(m_operatorController, HIDConstants.kRB);
    //offLED = new JoystickButton(m_operatorController, HIDConstants.kX);
    stowArm = new JoystickButton(m_operatorController, HIDConstants.kA);
    topGridArm = new JoystickButton(m_operatorController, HIDConstants.kY);
    midGridArm = new JoystickButton(m_operatorController, HIDConstants.kX);
    floorArm = new JoystickButton(m_operatorController, HIDConstants.kB);
    clawIn= new JoystickButton(m_operatorController, HIDConstants.kLB);
    clawOut= new JoystickButton(m_operatorController, HIDConstants.kRB);

    //orangeLED = new JoystickButton(m_driverController, HIDConstants.kY);

    coneTarget.whileTrue(new TargetCommand(m_driveSubsystem, m_visionTargeting, 0));
    cubeTarget.whileTrue(new TargetCommand(m_driveSubsystem, m_visionTargeting, 1));
    tapeTarget.whileTrue(new TargetCommand(m_driveSubsystem, m_visionTargeting, 3));
    //aprilTag.whileTrue(new TargetCommand(m_driveSubsystem, m_visionTargeting, 2));
    //xBalance.whileTrue(new Balance(m_driveSubsystem, true));
    yBalance.whileTrue(new Balance(m_driveSubsystem, false));
    zeroDrive.whileTrue(new ZeroSwerveCommand(m_driveSubsystem));

    //runClaw.whileTrue(new RunClawCommand(m_claw, "stop"));

   // purpleLED.whileTrue(new SetColor(m_leds, "purple"));
   // yellowLED.whileTrue(new SetColor(m_leds, "yellow"));
    //offLED.whileTrue(new SetColor(m_leds, "off"));

    stowArm.whileTrue(new ArmCommand(m_shoulder, m_wrist, "STOWED"));
    topGridArm.whileTrue(new ArmCommand(m_shoulder, m_wrist, "TOPGRID"));
    midGridArm.whileTrue(new ArmCommand(m_shoulder, m_wrist, "MIDGRID"));
    floorArm.whileTrue(new ArmCommand(m_shoulder, m_wrist, "FLOOR"));
    clawIn.whileTrue(new RunClawCommand(m_claw, "FORWARD"));
    clawOut.whileTrue(new RunClawCommand(m_claw, "BACKWARD"));

    //orangeLED.whileTrue(new SetColor(m_leds, "orange"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
   

    //return autoBuilder.fullAuto(pathGroup);
    //return auto.getSelected();
    return null;
  }
}

