package frc.util;

import java.util.TreeMap;


public class ShooterData {
    
    private static ShooterData instance;

    private static final TreeMap<Double, Double[]> data = new TreeMap<>();

    private ShooterData() {
        // add(23.34, 7500, 0.24);
        // add(18.06, 7500, 0.24);
        // add(13.97, 7750, 0.25);
        // add(10.6, 7750, 0.26);
        // add(8.89, 7500, 0.27);
        // add(3.69, 7500, 0.27);
        // add(0.11, 7750, 0.3);
        // add(-1.65, 8000, 0.28);
        // add(-3.1, 8250, 0.28);
        // add(-5.18, 8500, 0.28);
        // add(-6.5, 8800, 0.3);
        // add(-7.8, 8800, 0.3);
        // add(-9.1, 9000, 0.3);
        // add(-12.06, 9250, 0.32);

        add(23.34, 7800, 0.24);
        add(18.06, 7800, 0.24);
        add(13.97, 8050, 0.25);
        add(10.6, 8050, 0.26);
        add(8.89, 7650, 0.27);
        add(3.69, 7800, 0.27);
        add(0.11, 7900, 0.3);
        add(-1.65, 8100, 0.28);
        add(-3.1, 8200, 0.3);
        add(-4.35, 8200, 0.3);
        add(-5.18, 8500, 0.3);
        add(-6.5, 8800, 0.3);
        add(-7.8, 9000, 0.3);
        add(-9.1, 9300, 0.3);
        add(-12.06, 9550, 0.32);
        
    }

    public static ShooterData getInstance() {
        if (instance == null) {
            instance = new ShooterData();
        }

        return instance;
    }

    public Double[] getEntry(double ty) {
        Double[] empty = { 0.0, 0.0 };

        try {
            Double ceiling = data.ceilingKey(ty);
            Double floor = data.floorKey(ty);

            if (ceiling != null && floor != null) {
                return Math.abs(ceiling - ty) < Math.abs(floor - ty) ? data.get(ceiling) : data.get(floor);
            } else if (ceiling != null) {
                return data.get(ceiling);
            } else if (floor != null) {
                return data.get(floor);
            } else {
                return empty;
            }
        } catch(Exception e) {
            return empty;
        }
    }

    public double getShooterSpeed(double ty) {
        return getEntry(ty)[0];
    }

    public double getHoodAngle(double ty) {
        return getEntry(ty)[1];
    }

    private void add(double ty, double speed, double angle) {
        Double[] speedAngle = { speed, angle };

        data.put(ty, speedAngle);
    }
}
