// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.camera;

import edu.wpi.first.cscore.UsbCamera;

/** Add your docs here. */
public class HardwareCameraFactory {
    public static CameraImpl makeCameraInstance() {
        return new CameraImpl(0);
    }
}
