package net.ironpulse.utils;

import net.ironpulse.utils.TunableNumber;
import net.ironpulse.utils.ShootingParameters;

import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.TreeMap;

public class ShootingParametersTable {
    private final List<ParametersBinding> parameters = new ArrayList<>();
    private final NavigableMap<Double, ShootingParameters> interpolatingTable = new TreeMap<>();

    private static ShootingParametersTable instance;

    public static ShootingParametersTable getInstance() {
        if (instance == null) {
            instance = new ShootingParametersTable();
        }
        return instance;
    }

    private ShootingParametersTable() {
        loadParameter(1.28, -7, 12);
        loadParameter(1.47, -7, 22);
        loadParameter(1.86, -8, 33);
        loadParameter(2.17, -9, 44);
        loadParameter(2.47, -9, 53);
        loadParameter(2.65, -10, 55);
        loadParameter(3.01, -11, 61);
        loadParameter(3.3, -12, 67);
        loadParameter(3.5, -12, 68);
        loadParameter(3.76, -13, 69.5);
        loadParameter(4, -13, 71);
        readyTuning();
    }

    private void loadParameter(double distance, double voltage, double angle) {
        interpolatingTable.put(distance, new ShootingParameters(voltage, angle));
    }

    private void readyTuning() {
        int counter = 1;
        for (Double key : interpolatingTable.keySet()) {
            parameters.add(new ParametersBinding(
                    new TunableNumber("P" + counter + " Distance", key),
                    new TunableNumber("P" + counter + " Voltage", interpolatingTable.get(key).getVoltage()),
                    new TunableNumber("P" + counter + " Angle", interpolatingTable.get(key).getAngle())));
            counter++;
        }
    }

    public void update() {
        interpolatingTable.clear();
        for (ParametersBinding bind : parameters) {
            interpolatingTable.put(bind.distance.get(),
                    new ShootingParameters(bind.shootingVoltage.get(), bind.shootingAngle.get()));
        }
    }

    public ShootingParameters getParameters(double distance) {
        if (distance <= interpolatingTable.firstKey()) {
            return interpolatingTable.firstEntry().getValue();
        }

        if (distance >= interpolatingTable.lastKey()) {
            return interpolatingTable.lastEntry().getValue();
        }

        Entry<Double, ShootingParameters> floor = interpolatingTable.floorEntry(distance);
        Entry<Double, ShootingParameters> ceiling = interpolatingTable.ceilingEntry(distance);

        double k = (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey());
        return new ShootingParameters(
                floor.getValue().getVoltage() + (ceiling.getValue().getVoltage() - floor.getValue().getVoltage()) * k,
                floor.getValue().getAngle() + (ceiling.getValue().getAngle() - floor.getValue().getAngle()) * k);
    }

    private class ParametersBinding implements Comparable<ParametersBinding> {
        public TunableNumber distance;
        public TunableNumber shootingAngle;
        public TunableNumber shootingVoltage;

        public ParametersBinding(TunableNumber distance, TunableNumber voltage, TunableNumber angle) {
            this.distance = distance;
            this.shootingAngle = angle;
            this.shootingVoltage = voltage;
        }

        @Override
        public int compareTo(ParametersBinding bind) {
            if (distance.get() - bind.distance.get() > 0) {
                return 1;
            } else if (distance.get() - bind.distance.get() < 0) {
                return -1;
            } else {
                return 0;
            }
        }
    }
}
