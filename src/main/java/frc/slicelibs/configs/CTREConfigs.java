// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.slicelibs.configs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants;


/** Add your docs here. */
public class CTREConfigs {

    /* Initiating the different TalonFX configurators */
    public final TalonFXConfiguration m_swerveDriveConfigs = new TalonFXConfiguration();
    public final TalonFXConfiguration m_swerveTurnConfigs = new TalonFXConfiguration();

    public CTREConfigs(){


        ////////////////////////////
        /// Swerve Drive Configs ///
        ////////////////////////////
        
        var swerveDrivePIDs = m_swerveDriveConfigs.Slot0;
        swerveDrivePIDs.kP = Constants.DriveConstants.DRIVE_KP;
        swerveDrivePIDs.kI = Constants.DriveConstants.DRIVE_KI;
        swerveDrivePIDs.kD = Constants.DriveConstants.DRIVE_KD;


        //////////////////////////
        /// Swerve Turn Configs //
        //////////////////////////
        
        var swerveTurnPIDs = m_swerveTurnConfigs.Slot0;
        swerveTurnPIDs.kP = Constants.DriveConstants.TURN_KP;
        swerveTurnPIDs.kI = Constants.DriveConstants.TURN_KI;
        swerveTurnPIDs.kD = Constants.DriveConstants.TURN_KD;

        m_swerveTurnConfigs.Feedback.SensorToMechanismRatio = Constants.DriveConstants.ANGLE_GEAR_RATIO;
        //m_swerveTurnConfigs.ClosedLoopGeneral.ContinuousWrap = true;

    }
    

}
