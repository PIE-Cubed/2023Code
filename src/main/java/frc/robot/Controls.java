package frc.robot;

/**
 * Imports
 */
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Start of class
 */
public class Controls {
    // CONSTANTS
    private final int JOYSTICK_ID = 1;
    private final int XBOX_ID     = 0;

    // Controller object declaration
    private Joystick       joystick;
    private XboxController xboxController;

    // Rate limiters
    private SlewRateLimiter xLimiter;

    //Constructor
    public Controls() {
        // Instance Creation
        joystick       = new Joystick(JOYSTICK_ID);
        xboxController = new XboxController(XBOX_ID);

        // Rate limiter
        xLimiter = new SlewRateLimiter(0.75);
    }

    /**
     * JOYSTICK FUNCTIONS
     */

    /**
     * DRIVE FUNCTIONS
     */

    /**
     * Positive values are from clockwise rotation and negative values are from counter-clockwise
     * @return rotatePower
     */
    public double getRotatePower() {
        double power = joystick.getZ(); 

        // If we are in deadzone or strafelock is on, rotatepower is 0
        if ((Math.abs(power) < 0.3) || (getStrafeLock() == true)) {
            power = 0;
        }

        // Cubes the power and clamps it because the rotate is SUPER sensitive
        power = Math.pow(power, 3.0);
        power = MathUtil.clamp(power, -.5, .5);
            
        return power;    
    }

    /**
     * Gets the drive X
     * @return driveX
     */
    public double getDriveX() {
        double power = joystick.getX();

        // Strafe lock removes deadzone and cubes power for more precision
        if (getStrafeLock() == true) {
            power = Math.pow(power, 3);
        }

        // If we are in deadzone or rotatelock is on, x is 0
        if ((Math.abs(power) < 0.05) || (getRotateLock() == true)) {
            power = 0;
        }

        // Prevents us from accelerating sideways too quickly
        power = xLimiter.calculate(power);
 
        return power;
    }

    /**
     * Gets the drive Y
     * @return driveY
     */
    public double getDriveY() {
        double power = joystick.getY() * -1;

        // Strafe lock removes deadzone and cubes power for more precision
        if (getStrafeLock() == true) {
            power = Math.pow(power, 3);
        }
        // If we are in deadzone or rotatelock is on, y is 0
        else if ((Math.abs(power) < 0.05) || (getRotateLock() == true)) {
            power = 0;
        }
        return power;
    }

    /**
     * Checks if we are in strafe lock mode
     * @return joystick button 5
     */
    private boolean getStrafeLock() {
        return joystick.getRawButton(5);
    }

    /**
     * Checks if we are in rotate lock mode
     * @return joystick button 3
     */
    private boolean getRotateLock() {
        return joystick.getRawButton(3);
    }

    /*
    public boolean toggleFieldDrive() {
        return joystick.getRawButton(10);
    }*/


    /**
     * XBOX FUNCTIONS
     */

    /**
     * Checks if the start button is pressed
     * @return start button pressed
     */
    public boolean autoKill() {
        return xboxController.getStartButtonPressed();
    }

}

// End of the Controls class