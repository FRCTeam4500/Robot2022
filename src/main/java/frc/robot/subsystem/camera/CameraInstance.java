package frc.robot.subsystem.camera;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

/** This class holds the camera on the intake of the 2022 robot 
 * The camera server can be accessed on shuffleboard, does not specifically need to be sent
*/
public class CameraInstance {

    private int id;

    public CameraInstance(int id) {
        this.id = id;
    }

    /** creates a simple or complex stream based on type defined in constructor */
    public void start() {
        createSimpleStream();
    }

    /** starts a camera server automaically without changing any default values */
    private void createSimpleStream() {
        CameraServer.startAutomaticCapture(id);
    }

    /** see robot2019/src/main/java/frc/robot/utility/CameraInstance.java
     * createComplexStream()
     * I believe this makes a crosshair on the camera, we dont need this but if we need an example
     * use that.
     */

}
