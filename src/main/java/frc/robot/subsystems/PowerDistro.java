/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Daniel Pearson
 * @version 2/20/2020
 */
public class PowerDistro extends SubsystemBase {
  private final PowerDistributionPanel panel;
  
  double temp;
  double energy;

  public PowerDistro() {
    panel = new PowerDistributionPanel();
    panel.clearStickyFaults();

    LiveWindow.disableTelemetry(panel);
  }

  @Override
  public void periodic() {
    temp = panel.getTemperature();
    energy = panel.getTotalEnergy();

    SmartDashboard.putNumber("Temperature", temp);
    SmartDashboard.putNumber("Energy", energy);
  }
}
