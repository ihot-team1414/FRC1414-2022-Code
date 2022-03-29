public class ShooterData {
    
    private static ShooterData instance;

    private static final TreeMap<double, double[]> data = TreeMap<>()

    public static ShooterData getInstance() {
        if (instance == null) {
            instance = ShooterData();
        }

        return instance;
    }

    private ShooterData() {
        add(-6, 8000, 0.35);
    }

    private void add(double dy, double speed, double angle) {
        data.put(dy, { speed, angle });
    }
}
