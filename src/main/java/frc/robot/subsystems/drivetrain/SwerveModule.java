package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
 
    private int driveCanID;
    private int turnCanId;

    private int module_number;

    private TalonFX drivingMotor;
    private TalonFX turningMotor;

    private AnalogEncoder rotationAnalogEncoder;

    private double chassisAngularOffset;
    private Rotation2d angularOffset;


    private SwerveModuleState desiredState = new SwerveModuleState(0, new Rotation2d());

    public SwerveModule(int modNum, int driveCanID,int turnCanId, double chassisOffset, Rotation2d offset ){
        module_number = modNum;
        this.driveCanID = driveCanID;
        this.turnCanId = turnCanId;

        drivingMotor = new TalonFX(driveCanID);
        turningMotor = new TalonFX(turnCanId);

        drivingMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.m_swerveDriveConfigs);
        turningMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.m_swerveTurnConfigs);
        


     //   rotationAnalogEncoder = new AnalogEncoder(0, 360, 0); //TODO get channel

    }

    public SwerveModuleState getSwerveModuleState(){
        return new SwerveModuleState(drivingMotor.getPosition().getValueAsDouble(), 
            new Rotation2d(turningMotor.getPosition().getValueAsDouble()));
    }
    
    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(drivingMotor.getPosition().getValueAsDouble(),
            new Rotation2d(turningMotor.getPosition().getValueAsDouble()));
    }

    public void setDutyCycle(double drive, double turn){
        drivingMotor.set(drive);
        turningMotor.set(turn);
    }

    public void setDesiredState(SwerveModuleState desiredState){
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle;

        correctedDesiredState.optimize(new Rotation2d(turningMotor.getPosition().getValueAsDouble()));

        VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

        drivingMotor.setControl(velocityRequest.withVelocity(correctedDesiredState.speedMetersPerSecond / 0.0508));

        PositionVoltage positionRequest = new PositionVoltage(correctedDesiredState.angle.getRotations());

        turningMotor.setControl(positionRequest.withPosition(correctedDesiredState.angle.getRotations()));


        


        this.desiredState = desiredState;

    }

    // TODO : add a RESET ENCODERS METHOD


}
