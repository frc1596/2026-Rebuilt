package frc.robot.subsystems;

import  java.util.function.BooleanSupplier;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
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
    private final  SparkMax intakePivot = new SparkMax(14, MotorType.kBrushless);
    private final SparkMax intakepivothelper = new SparkMax(37, MotorType.kBrushless);
    private final  SparkMax intakeFuel = new SparkMax(13, MotorType.kBrushless);
    private final  SparkMax intakeFuelHelper = new SparkMax(16, MotorType.kBrushless);

    SparkMaxConfig intakePivotConfig = new SparkMaxConfig();
    SparkMaxConfig intakepivothelperconfig = new SparkMaxConfig();
    SparkMaxConfig intakeFuelConfig = new SparkMaxConfig();

//    private final RelativeEncoder mIntakeEncoder;
    private final RelativeEncoder mIntakeEncoder;
    private final RelativeEncoder mintakehelperencoder;

    private final SparkClosedLoopController mIntakePID;
   // private final SparkClosedLoopController mintakehelperPID;

    private final TrapezoidProfile m_pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(50, 100));
    private TrapezoidProfile.State m_pivotGoal = new TrapezoidProfile.State(0,0); 
    private TrapezoidProfile.State m_pivotSetpoint = new TrapezoidProfile.State(0,0);
 
    private static double kDt = 0.02;

    public IntakeSubsystem()
    {
        //intakepivitconfig
        intakePivotConfig.idleMode(IdleMode.kBrake);
       // intakePivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pidf(2.0,0,0,0); //Deprecated. Use ClosedLoopConfig.feedForward to set feedforward gains
             intakePivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(2.7,0,0,0); //Deprecated. Use ClosedLoopConfig.feedForward to set feedforward gains

    //     intakePivotConfig.closedLoop.positionWrappingEnabled(true);
    //   intakePivotConfig.closedLoop.positionWrappingMaxInput(1);
    //   intakePivotConfig.closedLoop.positionWrappingMinInput(0);
        intakePivotConfig.encoder.positionConversionFactor(1); 
        intakePivotConfig.encoder.velocityConversionFactor(1); 
       // intakePivotConfig.encoder.inverted(false);
        intakePivotConfig.smartCurrentLimit(35);
        intakePivotConfig.inverted(false); 
        intakePivot.configure(intakePivotConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        mIntakeEncoder = intakePivot.getEncoder();
          //     mIntakeEncoder = intakePivot.getAbsoluteEncoder();
        mIntakeEncoder.setPosition(0); 
        mIntakePID = intakePivot.getClosedLoopController(); 

         intakepivothelperconfig.idleMode(IdleMode.kBrake);
       // intakePivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pidf(2.0,0,0,0); //Deprecated. Use ClosedLoopConfig.feedForward to set feedforward gains
             intakepivothelperconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(2.9,0,0,0); //Deprecated. Use ClosedLoopConfig.feedForward to set feedforward gains
             intakepivothelperconfig.follow(14,true);
      intakepivothelperconfig.encoder.positionConversionFactor(1); 
        intakepivothelperconfig.encoder.velocityConversionFactor(1); 
        intakepivothelperconfig.smartCurrentLimit(35);
        intakepivothelperconfig.inverted(false); 
        intakepivothelper.configure(intakepivothelperconfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        mintakehelperencoder = intakepivothelper.getEncoder();

          //     mIntakeEncoder = intakePivot.getAbsoluteEncoder();
        mintakehelperencoder.setPosition(0); 
      //  mintakehelperPID = intakePivot.getClosedLoopController(); 

        intakeFuelConfig.idleMode(IdleMode.kCoast);
        intakeFuelConfig.encoder.positionConversionFactor(1); 
        intakeFuelConfig.encoder.velocityConversionFactor(1); 
        intakeFuelConfig.smartCurrentLimit(50);
        intakeFuelConfig.inverted(false); 

        intakeFuel.configure(intakeFuelConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        intakeFuelHelper.configure(intakeFuelConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    }


    @Override
    public void periodic() 
    {
        // Set point for the pivot
        m_pivotSetpoint = m_pivotProfile.calculate(kDt, m_pivotSetpoint, m_pivotGoal);

        //Set new position to the PID Controller
     //   mIntakePID.setReference(m_pivotSetpoint.position, com.revrobotics.spark.SparkBase.ControlType.kPosition); 
 mIntakePID.setSetpoint(m_pivotSetpoint.position,com.revrobotics.spark.SparkBase.ControlType.kPosition);
//  mintakehelperPID.setSetpoint(m_pivotSetpoint.position,com.revrobotics.spark.SparkBase.ControlType.kPosition);
  
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
        return Math.abs(m_pivotSetpoint.position - m_pivotGoal.position) < 0.2;
    }

    public Command intakePivot(double angle)
    {
        return this.startEnd(() -> setPivotAngle(angle), () -> doNothing()).until(() -> movePivotInPosition());
    }

    public void setFuelIntakeSpeed(double speed)
    {
        intakeFuel.set(-speed); 
        intakeFuelHelper.set(speed);
    }
 
    public double getFuelIntakeSpeed()
    {
        return(intakeFuel.get());
    }

public Command manualPivot(double angle){

    return this.runOnce(()->m_pivotGoal.position = m_pivotGoal.position + angle);
}
    public Command startFuelIntakeCmd(double speed)
    {
        return this.startEnd(() -> setFuelIntakeSpeed(speed),() -> setFuelIntakeSpeed(0.0));
    }

        public Command startFuelIntakeCmdAuto(double speed)
    {
        return this.runOnce(() -> setFuelIntakeSpeed(speed));
    }
}