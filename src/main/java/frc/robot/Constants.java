// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.LEDs;
import lib.LEDColor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    public static final class HIDConstants 
    {
        /**
         * XBox Controller layout
         * kA = 1;
         * kB = 2;
         * kX = 3; 
         * kY = 4;
         * kLB = 5;
         * kRB = 6;
         * kSelect = 7;
         * kStart = 8;
         * kLeftStickPress = 9;
         * kRightStickPress = 10;
         */

        public static final int k_DriverControllerPort = 0;
        public static final int k_OperatorControllerPort = 1;
        public static final double kDeadBand = 0.1;
        public static final int kA = 1;
        public static final int kB = 2;
        public static final int kX = 3;
        public static final int kY = 4;
        public static final int kLB = 5;
        public static final int kRB = 6;
        public static final int kBack = 7;
        public static final int kStart = 8;
        public static final int kDL = 270;
        public static final int kDR = 90;
        public static final int kDD = 180;
        public static final int kDU = 0;
    }

    public static final class SwerveDriveModuleConstants
    {
        public static enum k_SwerveDriveModules
        {
            leftFront,
            rightFront,
            leftRear,
            rightRear,
        };

        // Distance between front and back wheels on robot; unit is meters
        public static final double k_WheelBase = Units.inchesToMeters(24.25);

        public static final double k_RobotRadius = Units.inchesToMeters(34.295) / 2; 

        public static final double k_wheelDiameter = Units.inchesToMeters(3.9375); //meters

        /*TODO for all of these change when robot is characterized*/
        public static final double k_MaxTeleSpeed = Units.feetToMeters(16.3); //m/s checked :)
        public static final double k_MaxAutoSpeed = Units.feetToMeters(7); //m/s
        public static final double k_MaxAcceleration = 1; //m/s/s 
        
        public static final double k_XYjoystickCoefficient = 1; //speed limiter
        public static final double k_MaxAngularSpeed = Units.feetToMeters(16.3) / k_RobotRadius; // 628; //rad/s TODO confirm
        public static final double k_RotCoefficient = 1; //speed limiter

        public static final double k_driveEncoderTicksPerRotation = 2048; 
        public static final double k_gearRatio = 6.75;
        public static final double k_temperatureLimit = 110.00;

        public static final double k_steerFeedbackCoefficient = 0.087890625;

        public static final double k_balanceDeadband = 1;

        //public static double fieldCalibration = 0;

        //Angle offsets
        public static double leftFrontOffset =  14.85 + 180;
        public static double rightFrontOffset = 239.59 - 180;
        public static double leftRearOffset = 239.06;
        public static double rightRearOffset = 103.01 + 180;
    
        /*public static final SwerveDriveKinematics k_TeleKinematics =
        new SwerveDriveKinematics(
            new Translation2d(k_WheelBase / 2, -k_WheelBase / 2),
            new Translation2d(k_WheelBase / 2, k_WheelBase / 2),   
            new Translation2d(-k_WheelBase / 2, -k_WheelBase / 2),   
            new Translation2d(-k_WheelBase / 2, k_WheelBase / 2));
            */
        public static final SwerveDriveKinematics k_AutoKinematics =
        new SwerveDriveKinematics(
            new Translation2d(k_WheelBase / 2, k_WheelBase / 2),
            new Translation2d(k_WheelBase / 2, -k_WheelBase / 2),   
            new Translation2d(-k_WheelBase / 2, k_WheelBase / 2),   
            new Translation2d(-k_WheelBase / 2, -k_WheelBase / 2)); 
            
            public static final double k_pThetaController = 0;
            public static final double k_pTransController = 1;
            public static final double k_MaxAngularSpeedRadiansPerSecond = Math.PI;
            public static final double k_MaxAngularSpeedRadiansPerSecondSquared = Math.PI;
            public static final TrapezoidProfile.Constraints k_ThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                k_MaxAngularSpeedRadiansPerSecond, k_MaxAngularSpeedRadiansPerSecondSquared);
    }

    /**
     * The CAN Bus device IDs for devices used with the Swerve Drive: motor controllers and encoders
    */
    public static final class CANBusIDs
    { 
        public static final int k_LeftFront_DriveMotor = 1; 
        public static final int leftFrontCANCoderId = 9; 
        public static final int k_LeftFront_SteeringMotor = 2;

        public static final int k_RightFront_DriveMotor = 3; 
        public static final int rightFrontCANCoderId = 10; 
        public static final int k_RightFront_SteeringMotor = 4; 

        public static final int k_LeftRear_DriveMotor = 5; 
        public static final int leftRearCANCoderId = 11; 
        public static final int k_LeftRear_SteeringMotor = 6;

        public static final int k_RightRear_DriveMotor = 7; 
        public static final int rightRearCANCoderId = 12; 
        public static final int k_RightRear_SteeringMotor = 8; 

        public static final int k_pigeon2ID = 0;

        public static final int k_masterShoulderMotorID = 13;
        public static final int k_followerShoulderMotorID = 14;
        public static final int k_wristMotorID = 15;
        public static final int k_clawMotorID = 16;
    }

    public static class DriveModulePIDValues
    {
        public static double k_driveP = 3;
        public static double k_driveI = 0;
        public static double k_driveD = 0.0; 
        public static double k_driveFF = 1 / Units.feetToMeters(SwerveDriveModuleConstants.k_MaxTeleSpeed); 
        public static final int k_ToleranceInTicks = 5; 
    }

    public static class SteerModulePIDValues
    {
        public static double k_steerP = 3;
        public static double k_steerI = 0.0;
        public static double k_steerD = 0.50;

        public static final double k_steerDeadband = 0.02; // Deadband on the motor controller
        public static final int k_ToleranceInDegrees = 1;
    }

    public static class ClawSubsystemConstants
    {
        public static final double k_clawMotorSpeed = 0.6;
        public static final double k_clawReverseMotorSpeed = -0.6;
        public static final double k_clawStopMotorSpeed = 0.0;
        
        public static final double k_holdMotorSpeed = -0.5;
        public static final double k_holdReverseMotorSpeed = 0.75;
        public static final double k_holdStopMotorSpeed = 0.0;
    }
    
    public static class DigitalInputPort
    {
        public static final int k_shoulderEncoderPort = 0;
        public static final int k_wristEncoderPort = 1;
    }

    public static class MotorPIDValues
    {
        public static final double k_masterShoulderMotorP = 0; 
        public static final double k_masterShoulderMotorI = 0; 
        public static final double k_masterShoulderMotorD = 0; 
        public static final double k_masterShoulderMotorFF = 0; 

        public static final double k_followerShoulderMotorP = 0; 
        public static final double k_followerShoulderMotorI = 0; 
        public static final double k_followerShoulderMotorD = 0; 
        public static final double k_followerShoulderMotorFF = 0; 

        public static final double k_wristMotorP = 0; 
        public static final double k_wristMotorI = 0; 
        public static final double k_wristMotorD = 0; 
        public static final double k_wristMotorFF = 0; 

        public static final double k_clawMotorP = 0; 
        public static final double k_clawMotorI = 0; 
        public static final double k_clawMotorD = 0; 
        public static final double k_clawMotorFF = 0; 
    }
    public static class SetColorValues
    {
        public static final LEDColor kRed = new LEDColor(255, 0, 0);
        public static final LEDColor kGreen = new LEDColor(0, 255, 0);
        public static final LEDColor kBlue = new LEDColor(0, 0, 255);
        public static final LEDColor kYellow = new LEDColor(255, 255, 0);
        public static final LEDColor kPurple = new LEDColor(153, 51, 255);
        public static final LEDColor kOrange = new LEDColor(255, 128, 0);
        public static final LEDColor kOff = new LEDColor(0, 0, 0);
    }

    public static class LEDsSubsystemConstants
    {
        public static final int k_port = 4;
        public static final int k_numLeds = 300;
    }

} 
    
    


   


