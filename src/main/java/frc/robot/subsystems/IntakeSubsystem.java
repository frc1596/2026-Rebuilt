package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


public class IntakeSubsystem extends SubsystemBase{
    private final TalonFX intakeMotor;
    // public static DigitalInput coralSensor = new DigitalInput(7);
    //public static DigitalInput algaeSensor = new DigitalInput(2); 

    //public static final double INTAKE_SPEED = 0.5;

    public IntakeSubsystem()
    {
        //kraken setup
        intakeMotor = new TalonFX(16);
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true; 
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
        intakeMotor.getConfigurator().apply(intakeConfig, 0.05);
    }


    public void startIntake(double speed)
    {
        intakeMotor.set(speed);
    }

    public void stopIntake()
    {
        intakeMotor.set(0);
    }

    public double getIntakeSpeed()
    {
        return(intakeMotor.get());
    }

    public void reverseIntake(double speed)
    {
        intakeMotor.set(speed);
    }
  
    public void doNothing(){}

    public Command runIntakes(double speed)
    {
       return this.startEnd(() -> startIntake(speed), () -> stopIntake());
    }

    public Command runIntakesAuto(double speed)
    {
       return this.startEnd(() -> startIntake(speed), () -> Commands.waitSeconds(1));
    }

    public Command stopIntakes()
    {
        return this.run(() -> stopIntake());
    }

    public Command reverseIntakes(double speed)
    {
        return this.startEnd(() -> reverseIntake(speed), () -> reverseIntake(0.12));
    }

    public boolean getSensor()
    {
        // SmartDashboard.putBoolean("Sensor GetSensor:", coralSensor.get()); 
        //return coralSensor.get(); 
        return false;
    }
}

