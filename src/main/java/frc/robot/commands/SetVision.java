package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.LedModes;
import frc.robot.subsystems.Limelight.CamModes;

public class SetVision extends CommandBase {
  public Limelight limelight;

  public SetVision(Limelight vision) {
    limelight = vision;
    addRequirements(limelight);
  }

  @Override
  public void initialize() {
    limelight.setLed(LedModes.OFF);
    limelight.setCam(CamModes.DRIVER);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
