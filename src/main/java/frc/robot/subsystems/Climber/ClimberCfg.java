package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogInput;

public class ClimberCfg {
    public static final int CLIMBER_SOLENOID1_CHAN = 1;
    public static final int CLIMBER_SOLENOID2_CHAN = 2;
    public static final int CLIMBER_PSI_CHAN = 0;
     public static final double CLIMBER_PSI_LIMIT = 35;
    
    public static final Solenoid CLIMBER_SOLENOID1 = new Solenoid(PneumaticsModuleType.REVPH, CLIMBER_SOLENOID1_CHAN);
    public static final Solenoid CLIMBER_SOLENOID2 = new Solenoid(PneumaticsModuleType.REVPH, CLIMBER_SOLENOID2_CHAN);
    public static final AnalogInput CLIMBER_PSI = new AnalogInput(CLIMBER_PSI_CHAN);
   
    
}
