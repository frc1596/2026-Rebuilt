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
    private final  SparkMax intakePivot = new SparkMax(14, MotorType.kBrushless);
    private final  SparkMax intakeFuel = new SparkMax(13, MotorType.kBrushless);

    SparkMaxConfig intakePivotConfig = new SparkMaxConfig();
    SparkMaxConfig intakeFuelConfig = new SparkMaxConfig();

    private final RelativeEncoder mIntakeEncoder;
    private final SparkClosedLoopController mIntakePID;

    private final TrapezoidProfile m_pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(60, 200));
    private TrapezoidProfile.State m_pivotGoal = new TrapezoidProfile.State(0,0); 
    private TrapezoidProfile.State m_pivotSetpoint = new TrapezoidProfile.State(0,0);
 
    private static double kDt = 0.02;

    public IntakeSubsystem()
    {
        //intakepivitconfig
        intakePivotConfig.idleMode(IdleMode.kBrake);
        intakePivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(1.6,0,0,0); //Deprecated. Use ClosedLoopConfig.feedForward to set feedforward gains
        intakePivotConfig.encoder.positionConversionFactor(1); 
        intakePivotConfig.encoder.velocityConversionFactor(1); 
        intakePivotConfig.smartCurrentLimit(50);
        intakePivotConfig.inverted(false); 

        intakePivot.configure(intakePivotConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        mIntakeEncoder = intakePivot.getEncoder();
        mIntakeEncoder.setPosition(0); 
        mIntakePID = intakePivot.getClosedLoopController(); 

        intakeFuelConfig.idleMode(IdleMode.kCoast);
        intakeFuelConfig.encoder.positionConversionFactor(1); 
        intakeFuelConfig.encoder.velocityConversionFactor(1); 
        intakeFuelConfig.smartCurrentLimit(30);
        intakeFuelConfig.inverted(false); 

        intakeFuel.configure(intakeFuelConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    }


    @Override
    public void periodic() 
    {
        // Set point for the pivot
        m_pivotSetpoint = m_pivotProfile.calculate(kDt, m_pivotSetpoint, m_pivotGoal);

        //Set new position to the PID Controller
        mIntakePID.setReference(m_pivotSetpoint.position, com.revrobotics.spark.SparkBase.ControlType.kPosition); 
    }

    public void setPivotAngle(double angle)
    {
        m_pivotGoal = new TrapezoidProfile.State(angle, 0);
    }

    public void doNothing()
    {
        //literally does nothing for logic
    }

    public boolean movePivotInPosition() {
        return Math.abs(m_pivotSetpoint.position - m_pivotGoal.position) < 1.0;
    }

    public Command intakePivot(double angle)
    {
        return this.startEnd(() -> setPivotAngle(angle), () -> doNothing()).until(() -> movePivotInPosition());
    }

    public void setFuelIntakeSpeed(double speed)
    {
        intakeFuel.set(speed); 
    }
 
    public double getFuelIntakeSpeed()
    {
        return(intakeFuel.get());
    }

    public Command stopFuelIntake()
    {
        return this.run(()->stopFuelIntake());
    }

    public Command startFuelIntakeCmd(double speed)
    {
        return this.startEnd(() -> setFuelIntakeSpeed(speed), () -> setFuelIntakeSpeed(0));
    }
    
    // public Command runFuelIntakesAuto(double speed)
    // {
    //     return this.startEnd(() -> startFuelIntake(speed), () -> Commands.waitSeconds(1));
    // }
}