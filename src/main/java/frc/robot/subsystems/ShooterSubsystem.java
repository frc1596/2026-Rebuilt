package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{

    private final SparkMax spindexter = new SparkMax(51, MotorType.kBrushless);
    private final SparkMax turretHood = new SparkMax(50, MotorType.kBrushless);
    private final SparkMax turretRotate = new SparkMax(53, MotorType.kBrushless);
    private final TalonFX feeder = new TalonFX(52);
    private final TalonFX shootOne = new TalonFX(54);
    private final TalonFX shootTwo = new TalonFX(55);

    public ShooterSubsystem()
    {
        
        SparkMaxConfig spindexterConfig = new SparkMaxConfig();
        SparkMaxConfig turretHoodConfig = new SparkMaxConfig();
        SparkMaxConfig turretRotateConfig = new SparkMaxConfig();
    
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
            feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true; 
            feederConfig.CurrentLimits.SupplyCurrentLimit = 40;
            feeder.getConfigurator().apply(feederConfig, 0.05);

        TalonFXConfiguration shootConfig = new TalonFXConfiguration();
            shootConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            shootConfig.CurrentLimits.SupplyCurrentLimitEnable = true; 
            shootConfig.CurrentLimits.SupplyCurrentLimit = 40;
            shootOne.getConfigurator().apply(shootConfig, 0.05);
            shootTwo.getConfigurator().apply(shootConfig, 0.05);
        }
        
        public void setSpindexterSpeed(double speed)
        {
            spindexter.set(speed);
        }

        public void doNothing()
        {

        }
        
        public void setFeederSpeed(double speed)
        {
            feeder.set(speed);
        }

        public void setShootSpeed(double speed)
        {
            shootOne.set(speed);
            shootTwo.set(speed);
        }
        
        public double getShootOneSpeed()
        {
            return(shootOne.get());
        }

        public double getShootTwoSpeed()
        {
            return(shootTwo.get());
        }

        // it starts the shoot
        public void startShoot()
        {
            if(getShootOneSpeed()  > 10 &&  getShootTwoSpeed()>10)
            {
                // start everything else
                setSpindexterSpeed(1);
                setFeederSpeed(1);
            }
                else{
                    doNothing();
                }
        }


    }
    
    
