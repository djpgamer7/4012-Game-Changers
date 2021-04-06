/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
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

import java.io.IOException;
import java.nio.file.Path;
import java.sql.Driver;
import java.util.List;

/**
 * @author Daniel Pearson
 * @version 2/24/2020
 */

@SuppressWarnings("unused")
public class RobotContainer {
  private final DriveTrain drive = new DriveTrain();
  private final Limelight vision = new Limelight();
  private final Intake intake = new Intake();
  private final Pneumatics pneumatics = new Pneumatics();
  private final WheelOfPain wheel = new WheelOfPain();
  private final Shooter shooter = new Shooter();
  private final Hopper hopper = new Hopper();

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private final Joystick stickL = new Joystick(Constants.LEFT_JOY);
  private final Joystick stickR = new Joystick(Constants.RIGHT_JOY);
  //private DanController xboxCont = new DanController(Constants.MANIP_CONTROLLER);

  public RobotContainer() {
    // Configure the button bindings
    configDefaultCommands();
    configureButtonBindings();

    NetworkTableInstance.getDefault().setUpdateRate(.010);

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
    drive.setDefaultCommand(new TankDrive(drive, stickL::getY, stickR::getY));

    vision.setDefaultCommand(new SetVision(vision));
  
    pneumatics.setDefaultCommand(new RunCompressor(pneumatics));
  }

  public Command getAutonomousCommand() {

    var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                            Constants.AutoConstants.ksVolts,
                            Constants.AutoConstants.kvVoltSecondsPerMeter,
                            Constants.AutoConstants.kaVoltSecondsSquaredPerMeter
                    ),
                    Constants.AutoConstants.kDriveKinematics,
                    10
            );

    TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeed,
                    Constants.AutoConstants.kMaxAcceleration
            ).setKinematics(Constants.AutoConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    /*
    String searchAJson = "paths/SearchA.wpilib.json";

    Trajectory searchATrajectory = new Trajectory();


    try {
      Path galacticSearchAPath = Filesystem.getDeployDirectory().toPath().resolve(searchAJson);
      searchATrajectory = TrajectoryUtil.fromPathweaverJson(galacticSearchAPath);
    } catch (IOException e) {
      DriverStation.reportError("Unable to open trajectory: " + searchAJson, e.getStackTrace());
    }*/

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                    new Translation2d(1, 1),
                    new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
    );

    RamseteCommand initialCommand = new RamseteCommand(
            exampleTrajectory,
            drive::getPose,
            new RamseteController(Constants.AutoConstants.kRameseteB, Constants.AutoConstants.kRameseteZeta),
            new SimpleMotorFeedforward(
                    Constants.AutoConstants.ksVolts,
                    Constants.AutoConstants.kvVoltSecondsPerMeter,
                    Constants.AutoConstants.kaVoltSecondsSquaredPerMeter
            ),
            Constants.AutoConstants.kDriveKinematics,
            drive::getSpeeds,
            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
            drive::tankDriveVolts,
            drive
    );

    drive.resetOdometry(exampleTrajectory.getInitialPose());

    return initialCommand.andThen(() -> drive.tankDriveVolts(0, 0));

    /*
    RamseteCommand testCommand = new RamseteCommand(
            testTrajectory,
            drive::getPose,
            new RamseteController(Constants.AutoConstants.kRameseteB, Constants.AutoConstants.kRameseteZeta),
            new SimpleMotorFeedforward(
                    Constants.AutoConstants.ksVolts,
                    Constants.AutoConstants.kvVoltSecondsPerMeter,
                    Constants.AutoConstants.kaVoltSecondsSquaredPerMeter
                    ),
            Constants.AutoConstants.kDriveKinematics,
            drive::getSpeeds,
            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
            drive::tankDriveVolts,
            drive
    );
    *
     */

    //drive.resetSensors(testTrajectory.getInitialPose());

    //return testCommand.andThen(() -> drive.tankDriveVolts(0, 0));
  }
}
