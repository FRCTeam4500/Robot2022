// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.TankDrive;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.component.OutputComponent;


public class TankDrive extends SubsystemBase {
	private OutputComponent doubleLeft;
	private OutputComponent doubleRight;

	private double inX;


	private DifferentialDriveKinematics driveKinematic;
	private double maxWheelSpeed = 900d;
	/** Creates a new TankDrive. */
    public TankDrive(OutputComponent left, OutputComponent right) {
    	this.doubleLeft = left;
    	this.doubleRight = right;
    	this.driveKinematic = new DifferentialDriveKinematics(0.8);
		Shuffleboard.getTab("drive").addNumber("x", () -> {return inX;});
    }

	public void move(double x, double y, double w) {
		inX = x;
		double meanOfYAndZ = (x + w) / -4;
		ChassisSpeeds speeds = new ChassisSpeeds(meanOfYAndZ, 0, y);
		DifferentialDriveWheelSpeeds speedsTwo = driveKinematic.toWheelSpeeds(speeds);
		speedsTwo.desaturate(maxWheelSpeed);
		doubleLeft.setOutput(speedsTwo.leftMetersPerSecond);
		doubleRight.setOutput(speedsTwo.rightMetersPerSecond);
	}

	@Override
	public void periodic() {
    	// This method will be called once per scheduler run
	}

}
