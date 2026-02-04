package frc.robot.subsystems;


import org.deceivers.drivers.LimelightHelpers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Interpolable;
import frc.robot.util.InterpolatingDouble;

import org.deceivers.drivers.LimelightHelpers;



public class ShooterSubsystem extends SubsystemBase{

    private final SparkMax spindexter = new SparkMax(51, MotorType.kBrushless);
    private final SparkMax turretHood = new SparkMax(50, MotorType.kBrushless);
    private final SparkMax turretRotate = new SparkMax(53, MotorType.kBrushless);
    private final TalonFX feeder = new TalonFX(52);
    private final TalonFX shootOne = new TalonFX(54);
    private final TalonFX shootTwo = new TalonFX(55);

    private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shootMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>(null, null);
    static {
        shootMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(.67));//~6 ft
        shootMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(.67));//~9ft
        shootMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(.67));//~12 ft
        shootMap.put(new InterpolatingDouble(6.0), new InterpolatingDouble(.67));//~15 ft
        shootMap.put(new InterpolatingDouble(7.0), new InterpolatingDouble(.67));//~18 ft




    }
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
        
        public void setHoodAngle(double angle)
        {
            turretHood.set(angle);
        }
        
        public void setSpindexterSpeed(double speed)
        {
            spindexter.set(speed);
        }

        public void doNothing()
        {

        }

        public void rotateTurret(double angle)
        {
            turretRotate.set(angle);
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
            Pose3d position = LimelightHelpers.getCameraPose3d_TargetSpace("limelight");
            position.getX(); 
            position.getY(); 
            position.getZ(); 

            double hoodAngle = shootMap.get(new InterpolatingDouble(position.getZ())).value;
            
            if (getShootOneSpeed() == 0 && getShootTwoSpeed() == 0) // if the shooters have not been started
            {
                setShootSpeed(15); // start the shooter 
            }
            else if (getShootOneSpeed() > 10 &&  getShootTwoSpeed() > 10) // if the shooters are up to speed 
            {
                setSpindexterSpeed(1);  // start everything else
                setFeederSpeed(1);
            }
            else{
                doNothing();
            }
        }


    public Command shootCommand()
    {
        return this.startEnd(() -> startShoot(), () -> doNothing());
    }
}
    
    
