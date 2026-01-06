package frc.robot.util;

import edu.wpi.first.networktables.*;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;

public class GenericNTButton {
    private final String name;
    private boolean defaultValue;
    private final AtomicBoolean value = new AtomicBoolean(false);
    private final BooleanTopic topic;
    private final LoggedNetworkBoolean entry;
    private NetworkTable subTable;
    private final BooleanSupplier controllerConnected;

    public GenericNTButton(String name, NetworkTable table, boolean defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;

        controllerConnected = () -> false;

        topic = table.getBooleanTopic(name);
        topic.getEntry(defaultValue).setDefault(defaultValue);
        entry = new LoggedNetworkBoolean(name, defaultValue);

        NetworkTableInstance.getDefault().addListener(topic, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            value.set(event.valueData.value.getBoolean());
        });

        topic.publish();
    }

    public GenericNTButton(String name, NetworkTable table, BooleanSupplier controllerConnected, boolean defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;

        this.controllerConnected = controllerConnected;

        topic = table.getBooleanTopic(name);
        topic.getEntry(defaultValue).setDefault(defaultValue);
        entry = new LoggedNetworkBoolean(name, defaultValue);

        NetworkTableInstance.getDefault().addListener(topic, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            if (!controllerConnected.getAsBoolean()) {
                value.set(event.valueData.value.getBoolean());
            }
        });

        topic.publish();
    }

    public GenericNTButton(String name, NetworkTable table, String group, BooleanSupplier controllerConnected, boolean defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;

        this.controllerConnected = controllerConnected;

        subTable = table.getSubTable(group);
        topic = subTable.getBooleanTopic(name);
        topic.getEntry(defaultValue).setDefault(defaultValue);
        entry = new LoggedNetworkBoolean(subTable.getPath() + "/" + name, defaultValue);

        NetworkTableInstance.getDefault().addListener(topic, EnumSet.of(NetworkTableEvent.Kind.kValueLocal), event -> {
            value.set(event.valueData.value.getBoolean());
        });

        NetworkTableInstance.getDefault().addListener(topic, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> {
            if (!controllerConnected.getAsBoolean()) {
                value.set(event.valueData.value.getBoolean());
                subTable.getTopics().forEach(topic -> {
                    GenericEntry subTableEntry = topic.getGenericEntry();
                    if (!topic.getName().equals(name)) subTableEntry.setBoolean(false);
                });
                topic.getEntry(defaultValue).set(true);
            }
        });

        topic.publish();
    }

    public boolean get() {
        return value.get();
    }

    public BooleanEntry getTopic() {
        return topic.getEntry(defaultValue);
    }

    public void set(boolean value) {
        topic.getEntry(defaultValue).set(value);
    }

    public void setDefaultValue(boolean value) {
        defaultValue = (boolean) value;
    }

}
