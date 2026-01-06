package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.util.GenericNTButton;
import org.json.simple.parser.ParseException;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class DashboardController {

    private static class ControllerConfig {
        public String name;
        public String group;
        public int buttonID;
        public boolean defaultValue;

        public ControllerConfig() {};
        public ControllerConfig(String name, String group, int buttonID, boolean defaultValue) {
            this.name = name;
            this.group = group;
            this.buttonID = buttonID;
            this.defaultValue = defaultValue;
        }
    }

    private final GenericHID controller;

    private final NetworkTableInstance ntInstance;
    private final NetworkTable table;
    private final String name;
    private final BooleanSupplier controllerConnected;

    private final Map<String, GenericNTButton> buttonMap = new HashMap<String, GenericNTButton>();
    private final Map<Integer, String> buttonNameMap = new HashMap<Integer, String>();
    private final Map<String, Integer> buttonIDMap = new HashMap<String, Integer>();

    NetworkTableEntry ntEntry;

    public DashboardController(int controllerPort, String name) {
        controller = new GenericHID(controllerPort);
        this.name = name;
        controllerConnected = controller::isConnected;

        ntInstance = NetworkTableInstance.getDefault();
        table = ntInstance.getTable(name);
    }

    public void update() {
        if (controller.isConnected()) {
            for (int i = 0; i < controller.getButtonCount(); i++) {
                if (controller.getRawButtonPressed(i)) {
                    buttonMap.get(buttonNameMap.get(i)).set(true);
                }
            }
        }
    }

    public void addButton(String identifier, int buttonID, boolean defaultValue) {
        buttonMap.put(identifier, new GenericNTButton(identifier, table, controllerConnected, defaultValue));
        buttonNameMap.put(buttonID, identifier);
        buttonIDMap.put(identifier, buttonID);
    }

    public void addButton(String identifier, String group, int buttonID, boolean defaultValue) {
        buttonMap.put(identifier, new GenericNTButton(identifier, table, group, controllerConnected, defaultValue));
        buttonNameMap.put(buttonID, identifier);
        buttonIDMap.put(identifier, buttonID);
    }

    public void buildFromJSON() throws IOException, ParseException {
        ObjectMapper mapper = new ObjectMapper();
        File configFile = Filesystem.getDeployDirectory().toPath().resolve(this.name + "_config.json").toFile();
        List<ControllerConfig> configList = mapper.readValue(configFile, mapper.getTypeFactory().constructCollectionType(List.class, ControllerConfig.class));
        for (ControllerConfig config : configList) {
            if (config.group == null) {
                addButton(config.name, config.buttonID, config.defaultValue);
            } else {
                addButton(config.name, config.group, config.buttonID, config.defaultValue);
            }
        }
    }

    protected boolean isButtonPressed(int buttonID) {
        return buttonMap.get(buttonNameMap.get(buttonID)).get();
    }

     protected boolean isButtonPressed(String identifier) {
        return buttonMap.get(identifier).get();
    }

     protected int getNumberOfButtons() {
        return buttonNameMap.size();
    }

    public int getButtonID(String identifier) {
        return buttonIDMap.get(identifier);
    }

}
