package frc.robot.Subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
    public UsbCamera camera1;

    public Camera()
    {
        //camera1 = CameraServer.startAutomaticCapture(0);
        //camera2 = CameraServer.startAutomaticCapture(1);

        //camera1.setFPS(30);
        //camera1.setResolution(1152, 648);

        //camera2.setFPS(30);
        //camera2.setResolution(1152, 648);
        //test

        //camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        //camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    }

}
