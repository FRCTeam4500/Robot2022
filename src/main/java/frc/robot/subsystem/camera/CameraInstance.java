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
import edu.wpi.first.cscore.VideoMode;

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
        UsbCamera camera = CameraServer.startAutomaticCapture(id);
        camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, CameraConstants.width, CameraConstants.height, 32);
        new Thread(() -> {
            CvSink cvSink = CameraServer.getVideo();
            CvSource outputStream = CameraServer.putVideo("Camera Stream", CameraConstants.width, CameraConstants.height);
            Mat source = new Mat();
            while(!Thread.interrupted()) {
                cvSink.grabFrame(source);
                outputStream.putFrame(source);
            }
        }).start();
    }

    /** see robot2019/src/main/java/frc/robot/utility/CameraInstance.java
     * createComplexStream()
     * I believe this makes a crosshair on the camera, we dont need this but if we need an example
     * use that.
     */

}
