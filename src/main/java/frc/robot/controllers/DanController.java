package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID;

public class DanController extends Joystick {

    public DanController(int port){
        super(port);
    }

    public JoystickButton getXButton(){
        return new JoystickButton(this, 3);
    }
    
    public JoystickButton getYButton(){
        return new JoystickButton(this, 4);
    }
    
    public JoystickButton getAButton(){
        return new JoystickButton(this, 1);
    }
    
    public JoystickButton getBButton(){
        return new JoystickButton(this, 2);
    }
    
    
    public double getYAxis(GenericHID.Hand hand) {
        return this.getY(hand);
    }
                      
    public double getXAxis(GenericHID.Hand hand){
        return this.getX(hand);
    }                   
}
