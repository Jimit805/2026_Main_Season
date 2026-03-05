package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
 
    private int driveCanID;
    private int turnCanId;

    private int module_number;

    private TalonFX drivingMotor;
    private TalonFX turningMotor;

    private AnalogEncoder rotationAnalogEncoder;
  //  private AnalogInput rotation;
    private Encoder integratedEncoder;

    private double chassisAngularOffset;
    private Rotation2d angularOffset;


    private SwerveModuleState desiredState = new SwerveModuleState(0, new Rotation2d());

    public SwerveModule(int modNum, int driveCanID, int turnCanId,int encoderId, double chassisOffset, Rotation2d offset){
        module_number = modNum;
        this.driveCanID = driveCanID;
        this.turnCanId = turnCanId;
        angularOffset = offset;
        drivingMotor = new TalonFX(driveCanID);
        turningMotor = new TalonFX(turnCanId);

      //  rotation  = new AnalogInput(encoderId);

        drivingMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.m_swerveDriveConfigs);
        turningMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.m_swerveTurnConfigs);

        rotationAnalogEncoder = new AnalogEncoder(encoderId, 360, 0); //TODO get channel
        // rotationAnalogEncoder = new AnalogEncoder(encoderId);
        //turningMotor.setPosition(Math.abs(rotationAnalogEncoder.get() - (angularOffset.getDegrees())));
        //turningMotor.setPosition(turningMotor.getPosition().getValueAsDouble() - angularOffset.getDegrees());

        turningMotor.setPosition(rotationAnalogEncoder.get());
        //PositionVoltage initial = new PositionVoltage(rotationAnalogEncoder.get() * 360 - angularOffset.getDegrees());

        // turningMotor.setControl(initial.withPosition(Math.abs(rotationAnalogEncoder.get() * 360 - angularOffset.getDegrees())));
        turningMotor.setControl(new PositionVoltage(desiredState.angle.getRotations()));
    
        desiredState.angle = Rotation2d.fromRotations(rotationAnalogEncoder.get());

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

        // turningMotor.setControl(new PositionVoltage(desiredState.angle.getRotations()));
        //SmartDashboard.putNumber("Driving Encoder " + driveCanID, (rotationAnalogEncoder.get());
        


        this.desiredState = desiredState;

    }

    public double getTurnRotation(){
        return rotationAnalogEncoder.get();
    }
    public double getANYTHING(){
        return (Math.abs(rotationAnalogEncoder.get() - (angularOffset.getDegrees())));
    }

  //  public double getPosition() {
    //    double b = (inverted ? -1.0 : 1.0) * ((rotationAnalogEncoder.get()))
    //}
    
    // TODO : add a RESET ENCODERS METHOD

    public void resetEncoders(){
    }
    
    /*
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Back Left " + drivingCanID, (m_drivingEncoder.getPosition() - m_offset.getRotations()));
        SmartDashboard.putNumber("Encoder Angle "  + mod_number, m_intergratedTurningEncoder.getPosition());
    }*/
    

}


