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
        loadParameter(2.84, -10, 59.5);
        loadParameter(2.38, -7, 49.5);
        loadParameter(2.65, -8, 21.0);
        loadParameter(2.1, -7, 44);
        loadParameter(1.96, -7, 40);
        loadParameter(2.57, -7, 55.5);
        loadParameter(2.75, -9, 59);
        loadParameter(3.4, -12, 64);
        loadParameter(3.18, -11, 63);
        loadParameter(3.59, -12, 65.5);
        loadParameter(3.82, -12, 66.5);
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
