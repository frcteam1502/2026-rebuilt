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
        new LookupTablePoint(0, 1500, 14.5),
        new LookupTablePoint(2.42, 2250, 18),
        new LookupTablePoint(2.72, 2250, 19),
        new LookupTablePoint(3, 2100, 26),
        new LookupTablePoint(3.9, 2300, 28),
        new LookupTablePoint(4.6, 2450, 31),
        new LookupTablePoint(9.6, 4000, 33)
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

    public static final double[][] LOOKUP2 = {
        //{Speed,Angle}
        {1450,16},//0
        {1500,16},//0.25
        {1550,16},//0.5
        {1600,17},//0.75
        {1650,17},//1
        {1700,17},//1.25
        {1750,18},//1.5
        {1800,18},//1.75
        {1850,18},//2
        {1900,19},//2.25
        {1950,19},//2.5
        {2000,19},//2.75
        {2050,20},//3
        {2150,20},//3.25
        {2250,20},//3.5
        {2150,21},//3.75
        {2250,21},//4
        {2325,21},//4.25
        {2400,22},//4.5
        {2400,22},//4.75
        {2000,22},//5
        {2000,23},//5.25
        {2000,23},//5.5 WE STARTED HERE
        {2000,23},//5.75
        {2000,24},//6
        {2000,24},//6.25
        {2000,24},//6.5
        {2000,25},//6.75
        {2000,25},//7
        {2000,25},//7.25
        {2000,26},//7.5
        {2000,26},//7.75
        {2000,26},//8
        {2000,27},//8.25
        {2000,27},//8.5
        {2000,27},//8.75
        {2000,28},//9
        {2000,28},//9.25
        {2000,28},//9.5
        {2000,29},//9.75
        {2000,29},//10
        {2000,29},//10.25
        {2000,30},//10.5
        {2000,30},//10.75
        {2000,30},//11
        {2000,30},//11.25
        {2000,30},//11.5
        {2000,30},//11.75
        {2000,30},//12
        {2000,30},//12.25
        {2000,30},//12.5
        {2000,30},//12.75
        {2000,30},//13
        {2000,30},//13.25
        {2000,30},//13.5
        {2000,30},//13.75
        {2000,30},//14
        {2000,30},//14.25
        {2000,30},//14.5
        {2000,30},//14.75
        {2000,30},//15
        {2000,30},//15.25
        {2000,30},//15.5
        {2000,30},//15.75
        {2000,30},//16
        {2000,30},//16.25
        {2000,30},//16.5
        {2000,30}//16.75
    };
}
