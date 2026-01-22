package frc.robot.subsystems;

import  java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import com.revrobotics.RelativeEncoder;

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
        


    }

}