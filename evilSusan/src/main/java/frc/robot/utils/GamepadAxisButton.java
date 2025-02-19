package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GamepadAxisButton extends Trigger{

    public GamepadAxisButton(BooleanSupplier bs) {
        /**
	 * Create a gamepad axis for triggering commands as if it were a button.
	 *
	 * @param joystick     The GenericHID object that has the axis (e.g. Joystick, KinectStick,
	 *                     etc)
	 * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
	 * 
	 * @param threshold The threshold above which the axis shall trigger a command
	 */
        super(bs);
    }
    
}
