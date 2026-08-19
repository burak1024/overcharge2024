package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class TreeMaps {

    public static InterpolatingDoubleTreeMap ShooterVoltageMap() {
        InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();
        treeMap.put(-1.0, 3.0);
        treeMap.put(-4.0, 5.0);
        treeMap.put(-7.0, 9.0);
        treeMap.put(0.0, 2.0);
        treeMap.put(1.0, 3.0);
        treeMap.put(4.0, 5.0);
        treeMap.put(7.0, 9.0);
        return treeMap;
    }

    public static InterpolatingDoubleTreeMap HoodDegreeMap() {
        InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();
        treeMap.put(1.0, 0.3888888888888889);
        treeMap.put(2.0, 0.3333333333333333);
        treeMap.put(5.0, 0.1666666666666667);
        return treeMap;
    }

}
