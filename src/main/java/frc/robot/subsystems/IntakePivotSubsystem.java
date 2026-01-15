package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakePivotSubsystem extends SubsystemBase {
  
  private static double kDt = 0.02; //some sort of timing thing, don't touch probably

  private final SparkMax intakePivotSparkMax = new SparkMax(16, MotorType.kBrushless);
  SparkMaxConfig intakePivotConfig = new SparkMaxConfig();

  private final RelativeEncoder mIntakePivotEncoder;
  private final SparkClosedLoopController mIntakePID;

  private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(30, 25));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  public IntakePivotSubsystem() {

    intakePivotConfig.idleMode(IdleMode.kBrake);
    intakePivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(2.0, 0, 0.0); 
    intakePivotConfig.encoder.positionConversionFactor(1);//(360.0/(60.0));
    intakePivotConfig.encoder.velocityConversionFactor(1); //(360.0/(60.0*10));
    intakePivotConfig.smartCurrentLimit(50);
    intakePivotSparkMax.configure(intakePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

    //Set initial encoder position to 0
    mIntakePivotEncoder = intakePivotSparkMax.getEncoder();
    mIntakePivotEncoder.setPosition(0);
    mIntakePID = intakePivotSparkMax.getClosedLoopController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Calculate the set point for the pivot
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

    // New posistion to the PID controller
    mIntakePID.setReference(m_setpoint.position, com.revrobotics.spark.SparkBase.ControlType.kPosition);
  }

    // set target position for the pivot
    public void setAngle(double angle) {
        m_goal = new TrapezoidProfile.State(angle, 0);
    }

    public void doNothing(){}
    
    public Command intakePivot(double angle){
      return this.startEnd(() -> setAngle(angle), () -> doNothing()).until(() -> moveInPosistion());
    }

    public boolean moveInPosistion() {
      return Math.abs(m_setpoint.position - m_goal.position) < 1.0;
    }
}
