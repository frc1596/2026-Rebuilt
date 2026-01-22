package frc.robot.subsystems;

import  java.util.function.BooleanSupplier;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


public class IntakeSubsystem extends SubsystemBase{
    //Update once know pls - Ivy
    private final  SparkMax intakePivot = new SparkMax(00, MotorType.kBrushless);
    private final  SparkMax intakeFuel = new SparkMax(01, MotorType.kBrushless);

    SparkMaxConfig intakePivotConfig = new SparkMaxConfig();
    SparkMaxConfig intakeFuelConfig = new SparkMaxConfig();

    private final RelativeEncoder mIntakeEncoder;
    private final SparkClosedLoopController mIntakePID;

    private final TrapezoidProfile m_Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(40, 60));
    private TrapezoidProfile.State m_Goal = new TrapezoidProfile.State(0,0); 
    private TrapezoidProfile.State m_Setpoint = new TrapezoidProfile.State(0,0);
 
    public IntakeSubsystem()
    {
        //intakepivitconfig
        intakePivotConfig.idleMode(IdleMode.kBrake);
        intakePivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(0.4,0,0,0); //Deprecated. Use ClosedLoopConfig.feedForward to set feedforward gains
        intakePivotConfig.encoder.positionConversionFactor(1); 
        intakePivotConfig.encoder.velocityConversionFactor(1); 
        intakePivotConfig.smartCurrentLimit(40);
        intakePivotConfig.inverted(false); 

        intakePivot.configure(intakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mIntakeEncoder = intakePivot.getEncoder();
        mIntakeEncoder.setPosition(0); 
        mIntakePID = intakePivot.getClosedLoopController(); 

        intakeFuelConfig.idleMode(IdleMode.kBrake);
        intakeFuelConfig.encoder.positionConversionFactor(1); 
        intakeFuelConfig.encoder.velocityConversionFactor(1); 
        intakeFuelConfig.smartCurrentLimit(40);
        intakeFuelConfig.inverted(false); 

        intakeFuelConfig.follow(00);
        intakeFuel.configure(intakeFuelConfig, null, null); 

    }

}