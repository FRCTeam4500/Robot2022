/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

import frc.robot.subsystem.swerve.pathfollowingswerve.AdvancedSwerveControllerBuilder;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Add your docs here.
 */
public class GenericAutonUtilities {
    public static AdvancedSwerveControllerBuilder createDefaultControllerBuilder(){
        return new AdvancedSwerveControllerBuilder()
        .withInitialAllowableTranslationError(0.1)
        .withFinalAllowableTranslationError(0.1)
        .withAllowableRotationError(0.1)
        .withTranslationsEnabled(true)
        .with_kP(3)
        .with_kW(3)
        .withRotationsEnabled(true)
        .withEndRotation(new Rotation2d())
        .withMaxVelocity(2.4);
    }
}