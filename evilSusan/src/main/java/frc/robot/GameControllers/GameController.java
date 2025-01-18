package frc.robot.GameControllers;

import frc.robot.GameControllers.ControllerTypes;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GameController {
    
    ControllerTypes m_controllerType;
    public CommandXboxController xc;
    //public XboxController xc;

    public GameController(ControllerTypes controllerType){
        m_controllerType = controllerType;
        createXboxController();
    }
    private void createXboxController(){
        xc = new CommandXboxController(0);
        //xc = new XboxController(0);
        Trigger xButton = xc.x();
    };
}

