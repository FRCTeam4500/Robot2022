package frc.robot.subsystem.camera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

/** This class holds the camera on the intake of the 2022 robot 
 * The camera server can be accessed on shuffleboard, does not specifically need to be sent
*/
public class CameraImpl {

    private int id;

    public CameraImpl(int id) {
        this.id = id;
    }

    /** creates a simple or complex stream based on type defined in constructor */
    public void start() {
        createSimpleStream();
    }

    // starts a camera server automaically without changing any default values
    public void createSimpleStream() {
        UsbCamera camera = CameraServer.startAutomaticCapture(id);
        camera.setResolution(CameraConstants.width, CameraConstants.height);
        camera.setFPS(20);
    }



}
