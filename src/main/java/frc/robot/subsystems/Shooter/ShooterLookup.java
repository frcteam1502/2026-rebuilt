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
        new LookupTablePoint(1.8, 1825, 22),
        new LookupTablePoint(2.1, 1875, 26),
        new LookupTablePoint(2.5, 1875, 30),
        new LookupTablePoint(2.71, 1890, 30),
        new LookupTablePoint(2.82, 1900, 30),
        new LookupTablePoint(2.95, 1909, 30),
        new LookupTablePoint(3.15, 2100, 33),
        new LookupTablePoint(3.4, 2075, 33),
        new LookupTablePoint(3.8, 2175, 33),
        new LookupTablePoint(4.28, 2275, 33),
        new LookupTablePoint(4.6, 2425, 33),
        new LookupTablePoint(9.6, 4075, 33),
        new LookupTablePoint(14, 4575, 33)
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
