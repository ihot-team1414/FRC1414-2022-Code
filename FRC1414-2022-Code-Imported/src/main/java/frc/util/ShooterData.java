package frc.util;

import java.util.TreeMap;

public class ShooterData {
    
    private static ShooterData instance;

    private static final TreeMap<Double, Double[]> data = new TreeMap<>();

    private ShooterData() {
        add(-6, 8000, 0.35);
    }

    public static ShooterData getInstance() {
        if (instance == null) {
            instance = new ShooterData();
        }

        return instance;
    }

    public Double[] getEntry(double ty) {
        try {
            double ceiling = data.ceilingKey(ty);
            double floor = data.ceilingKey(ty);

            double closest = Math.abs(ceiling - ty) < Math.abs(floor - ty) ? ceiling : floor;

            return data.get(closest);
        } catch(Exception e) {
            Double[] empty = { 0.0, 0.0 };

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
