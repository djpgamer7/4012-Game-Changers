/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.io.*;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//Subsystems
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WheelOfPain;
import frc.robot.subsystems.Intake;
//Commands
import frc.robot.commands.TankDrive;
import frc.robot.commands.SetVision;
import frc.robot.commands.LockTarget;
import frc.robot.commands.RunCompressor;
import frc.robot.commands.IntakeManipulator;
import frc.robot.commands.SpinWheel;
import frc.robot.commands.ShootBall;

import frc.robot.controllers.DanController;

/**
 * @author Daniel Pearson
 * @version 2/24/2020
 */

@SuppressWarnings("unused")
public class RobotContainer {
  private DriveTrain drive = new DriveTrain();
  private Limelight vision = new Limelight();
  private Intake intake = new Intake();
  private Pneumatics pneumatics = new Pneumatics();
  private WheelOfPain wheel = new WheelOfPain();
  private Shooter shooter = new Shooter();
  private Hopper hopper = new Hopper();

  private SendableChooser<Command> chooser = new SendableChooser<>();

  private Joystick stickL = new Joystick(Constants.LEFT_JOY);
  private Joystick stickR = new Joystick(Constants.RIGHT_JOY);
  //private DanController xboxCont = new DanController(Constants.MANIP_CONTROLLER);

  public RobotContainer() {
    // Configure the button bindings
    configDefaultCommands();
    configureButtonBindings();

    /*
    chooser.setDefaultOption("Blue 1", new Blue1(drive, intake, vision, shooter, hopper));
    chooser.addOption("Blue 2", new Blue2(drive, intake, vision, shooter, hopper));
    chooser.addOption("Blue 3", new Blue3(drive, intake, vision, shooter, hopper));
    chooser.addOption("Red 1", new Red1(drive, intake, vision, shooter, hopper));
    chooser.addOption("Red 2", new Red2(drive, intake, vision, shooter, hopper));
    chooser.addOption("Red 3", new Red3(drive, intake, vision, shooter, hopper));  
    */
  }
  private void configureButtonBindings() {
    JoystickButton targetAcquired = new JoystickButton(stickL, 2);
    JoystickButton intakeButton = new JoystickButton(stickL, 4);
    JoystickButton shooterButton = new JoystickButton(stickL, 3);

    targetAcquired.whileHeld(new LockTarget(drive, vision));
    intakeButton.toggleWhenPressed(new IntakeManipulator(intake, hopper));
    shooterButton.whileHeld(new ShootBall(shooter, hopper));
  }

  public void configDefaultCommands(){
    drive.setDefaultCommand(new TankDrive(drive, 
    () -> stickL.getY(), 
    () -> stickR.getY()));

    vision.setDefaultCommand(new SetVision(vision));
  
    pneumatics.setDefaultCommand(new RunCompressor(pneumatics));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
