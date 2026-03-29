package frc.robot.subsystems.Shooter;

import java.util.List;

public class ShooterLookup {
    public static final class LookupTablePoint{
        double m_velocity;
        double m_hoodAngle;
        double m_distance;
    
        public LookupTablePoint(double meters, double velocity, double hoodAngle){
            this.m_hoodAngle = Math.toRadians(hoodAngle);
            this.m_distance = meters;
            this.m_velocity = velocity;
        }
        public LookupTablePoint(LookupTablePoint point){
            this.m_hoodAngle = point.m_hoodAngle;
            this.m_distance = point.m_distance;
            this.m_velocity = point.m_velocity;
        }
    }

    public static final List<LookupTablePoint> LookupTable = List.of(
        new LookupTablePoint(0, 1625, 14.5),
        new LookupTablePoint(1.8, 1775, 22),
        new LookupTablePoint(2.1, 1825, 26),
        new LookupTablePoint(2.5, 1825, 30),
        new LookupTablePoint(2.71, 1840, 30),
        new LookupTablePoint(2.82, 1850, 30),
        new LookupTablePoint(2.95, 1859, 30),
        new LookupTablePoint(3.15, 1925, 30),
        new LookupTablePoint(3.4, 1975, 30),
        new LookupTablePoint(3.8, 2125, 30),
        new LookupTablePoint(4.28, 2225, 33),
        new LookupTablePoint(4.6, 2375, 33),
        new LookupTablePoint(9.6, 4025, 33)
    );

    public static LookupTablePoint Lookup(double distance)
    {
        LookupTablePoint interpolated = new LookupTablePoint(LookupTable.get(0));

        for (int i = 1; i < LookupTable.size(); i++)
        {
            LookupTablePoint greater = LookupTable.get(i);

            if (LookupTable.get(i).m_distance > distance)  {
                interpolated.m_velocity =  interpolated.m_velocity + (distance - interpolated.m_distance )* (greater.m_velocity - interpolated.m_velocity)/(greater.m_distance-interpolated.m_distance);
                interpolated.m_hoodAngle = interpolated.m_hoodAngle + (distance - interpolated.m_distance) * (greater.m_hoodAngle - interpolated.m_hoodAngle)/(greater.m_distance-interpolated.m_distance);
                interpolated.m_distance = distance;
                return interpolated;

            }
            else {

                interpolated = new LookupTablePoint(LookupTable.get(i));
            }
        }

        return interpolated;
    }
}
