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
import frc.robot.commands.LockSwerveCommand;
import frc.robot.commands.RunClawCommand;
import frc.robot.commands.ScoreConeAndTaxi;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TargetCommand;
import frc.robot.commands.Taxi3;
import frc.robot.commands.TaxiAndBalance2;
import frc.robot.commands.TaxiAndGrabCube1Blue;
import frc.robot.commands.TaxiAndGrabCube1BlueNoBal;
import frc.robot.commands.TaxiAndGrabCube1Red;
import frc.robot.commands.TaxiAndGrabCube1RedNoBal;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem(true);
  public final VisionTargeting m_visionTargeting = new VisionTargeting();
  private final Claw m_claw = new Claw();
  private final Shoulder m_shoulder = new Shoulder();
  private final Wrist m_wrist = new Wrist();

  public final static LEDs m_leds = new LEDs(LEDsSubsystemConstants.k_port, LEDsSubsystemConstants.k_numLeds);
  XboxController m_driverController = new XboxController(HIDConstants.k_DriverControllerPort);
  XboxController m_operatorController = new XboxController(HIDConstants.k_OperatorControllerPort);


// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
  HashMap<String, Command> eventMap = new HashMap<>();
  
  private final SwerveDriveCommand m_driveCommand = new SwerveDriveCommand(m_driveSubsystem, m_driverController);
  
  public static JoystickButton coneTarget, cubeTarget, aprilTag, zeroDrive, lockDrive, purpleLED, offLED, yellowLED, runClaw, reverseClaw, topGridArmCone, midGridArmCone, floorArm, substationArm, clawIn, clawOut, topGridArmCube, midGridArmCube;

  SendableChooser<CommandBase> auto = new SendableChooser<CommandBase>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    configureButtonBindings();

    m_driveSubsystem.setDefaultCommand(m_driveCommand);
    m_driveSubsystem.resetIMU();
  
    configChooser();

    eventMap.put("ClawIn", new RunClawCommand(m_claw, "FORWARD"));
    eventMap.put("ClawOut", new RunClawCommand(m_claw, "BACKWARD"));
    eventMap.put("ClawStop", new RunClawCommand(m_claw, "STOP"));
    eventMap.put("ArmConeTop", new ArmCommand(m_shoulder, m_wrist, "TOPGRIDCONE"));
    eventMap.put("ArmStowed", new ArmCommand(m_shoulder, m_wrist, "STOWED"));
    eventMap.put("Wait2S", new WaitCommand(2));
    eventMap.put("ArmFloor", new ArmCommand(m_shoulder, m_wrist, "FLOOR"));
    eventMap.put("ArmCubeTop", new ArmCommand(m_shoulder, m_wrist, "TOPGRIDCUBE"));
    
  }

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

  private void configChooser()
  {
    //auto.addOption("DefaultTaxiAndMore", new DefaultTaxiAndMore(m_driveSubsystem, m_claw, m_shoulder, m_wrist));
    //auto.addOption("Score Cone and Taxi", new RunClawCommand(m_claw, "FORWARD").alongWith(new ArmCommand(m_shoulder, m_wrist, "TOPGRIDCONE")).withTimeout(2).andThen(autoBuilder.fullAuto(PathPlanner.loadPathGroup("ScoreConeAndTaxi", new PathConstraints(1.5, 1)))));
    //auto.addOption("Score Cone and Taxi", new ScoreConeAndTaxi(m_driveSubsystem, m_claw, m_shoulder, m_wrist));
    //auto.addOption("Mid", autoBuilder.fullAuto(PathPlanner.loadPathGroup("Position2", new PathConstraints(3, 2))));

    auto.addOption("B1 Score 2P and Bal", new SequentialCommandGroup(new ScoreConeAndTaxi(m_driveSubsystem, m_claw, m_shoulder, m_wrist, eventMap), new TaxiAndGrabCube1Blue(m_driveSubsystem, m_claw, m_shoulder, m_wrist, eventMap)));
    auto.addOption("R1 Score 2P and Bal", new SequentialCommandGroup(new ScoreConeAndTaxi(m_driveSubsystem, m_claw, m_shoulder, m_wrist, eventMap), new TaxiAndGrabCube1Red(m_driveSubsystem, m_claw, m_shoulder, m_wrist, eventMap)));
    auto.addOption("R2/B2 Score 1P and Bal", new SequentialCommandGroup(new ScoreConeAndTaxi(m_driveSubsystem, m_claw, m_shoulder, m_wrist, eventMap), new TaxiAndBalance2(m_driveSubsystem, m_claw, m_shoulder, m_wrist, eventMap)));
    auto.addOption("R3/B3 Score 1P and Taxi", new SequentialCommandGroup(new ScoreConeAndTaxi(m_driveSubsystem, m_claw, m_shoulder, m_wrist, eventMap), new Taxi3(m_driveSubsystem, m_claw, m_shoulder, m_wrist, eventMap)));
    auto.addOption("B1 Score 2P NO Bal", new SequentialCommandGroup(new ScoreConeAndTaxi(m_driveSubsystem, m_claw, m_shoulder, m_wrist, eventMap), new TaxiAndGrabCube1BlueNoBal(m_driveSubsystem, m_claw, m_shoulder, m_wrist, eventMap)));
    auto.addOption("R1 Score 2P NO Bal", new SequentialCommandGroup(new ScoreConeAndTaxi(m_driveSubsystem, m_claw, m_shoulder, m_wrist, eventMap), new TaxiAndGrabCube1RedNoBal(m_driveSubsystem, m_claw, m_shoulder, m_wrist, eventMap)));
    auto.addOption("Do Nothing", null);

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
    //aprilTag = new JoystickButton(m_driverController , HIDConstants.kBack);
    zeroDrive = new JoystickButton(m_driverController, HIDConstants.kStart);
    lockDrive = new JoystickButton(m_driverController, HIDConstants.kBack);
    purpleLED = new JoystickButton(m_driverController, HIDConstants.kLB);
    yellowLED = new JoystickButton(m_driverController, HIDConstants.kRB);
    //offLED = new JoystickButton(m_operatorController, HIDConstants.kX);
    topGridArmCone = new JoystickButton(m_operatorController, HIDConstants.kY);
    midGridArmCone = new JoystickButton(m_operatorController, HIDConstants.kX);
    floorArm = new JoystickButton(m_operatorController, HIDConstants.kB);
    clawIn= new JoystickButton(m_operatorController, HIDConstants.kLB);
    clawOut= new JoystickButton(m_operatorController, HIDConstants.kRB);
    topGridArmCube = new JoystickButton(m_operatorController, HIDConstants.kBack);
    midGridArmCube = new JoystickButton(m_operatorController, HIDConstants.kStart);
    substationArm = new JoystickButton(m_operatorController, HIDConstants.kA);

    //orangeLED = new JoystickButton(m_driverController, HIDConstants.kY);

    coneTarget.whileTrue(new TargetCommand(m_driveSubsystem, m_driverController, m_visionTargeting, 1, "limelight-b"));
    cubeTarget.whileTrue(new TargetCommand(m_driveSubsystem, m_driverController, m_visionTargeting, 0, "limelight-b"));
    zeroDrive.whileTrue(new ZeroSwerveCommand(m_driveSubsystem));
    lockDrive.whileTrue(new LockSwerveCommand(m_driveSubsystem));

    purpleLED.onTrue(new SetColor(m_leds, "purple"));
    yellowLED.onTrue(new SetColor(m_leds, "yellow"));
    //offLED.whileTrue(new SetColor(m_leds, "off"));

    topGridArmCone.onTrue(new ArmCommand(m_shoulder, m_wrist, "TOPGRIDCONE")).onFalse(new ParallelCommandGroup(new ArmCommand(m_shoulder, m_wrist, "STOWED"), new SetColor(m_leds, "off")));
    midGridArmCone.onTrue(new ArmCommand(m_shoulder, m_wrist, "MIDGRIDCONE")).onFalse(new ParallelCommandGroup(new ArmCommand(m_shoulder, m_wrist, "STOWED"), new SetColor(m_leds, "off")));
    topGridArmCube.onTrue(new ArmCommand(m_shoulder, m_wrist, "TOPGRIDCUBE")).onFalse(new ParallelCommandGroup(new ArmCommand(m_shoulder, m_wrist, "STOWED"), new SetColor(m_leds, "off")));
    midGridArmCube.onTrue(new ArmCommand(m_shoulder, m_wrist, "MIDGRIDCUBE")).onFalse(new ParallelCommandGroup(new ArmCommand(m_shoulder, m_wrist, "STOWED"), new SetColor(m_leds, "off")));
    substationArm.onTrue(new ArmCommand(m_shoulder, m_wrist, "SUBSTATION")).onFalse(new ParallelCommandGroup(new ArmCommand(m_shoulder, m_wrist, "STOWED"), new SetColor(m_leds, "off")));
    floorArm.onTrue(new ArmCommand(m_shoulder, m_wrist, "FLOOR")).onFalse(new ParallelCommandGroup(new ArmCommand(m_shoulder, m_wrist, "STOWED"), new SetColor(m_leds, "off")));
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
    return auto.getSelected();
    //return null;
  }
}

