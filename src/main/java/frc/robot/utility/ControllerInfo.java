package frc.robot.utility;

/**
 * A class representing controller configuration values
 * It is essentially a struct, so the members are all public
 * Its a reference type so it can be passed once then its members can be changed on the fly
 * Its not super standard java but I don't see anything wrong with it
 */

public class ControllerInfo {
    public double xDeadzone;
    public double yDeadzone;
    public double zDeadzone;
    public double xSensitivity;
    public double ySensitivity;
    public double zSensitivity;
}
