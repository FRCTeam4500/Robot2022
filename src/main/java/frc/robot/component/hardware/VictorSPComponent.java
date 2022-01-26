/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.component.hardware;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.component.OutputComponent;

/**
 * Add your docs here.
 */
public class VictorSPComponent extends VictorSP implements OutputComponent {

    public VictorSPComponent(int channel) {
        super(channel);
    }

    @Override
    public void setOutput(double output) {
        set(-output);
    }

    public double getOutput() {
        return -get();
    }

}