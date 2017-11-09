package de.monticar.lang.monticar.generator.python;

import java.util.ArrayList;
import java.util.HashMap;

public class RosTag {
    public String component = "";
    public ArrayList<RosInterface> subscriber = new ArrayList<>();
    public ArrayList<RosInterface> publisher = new ArrayList<>();
}

class RosInterface {
    public String type = "";
    public String topic = "";
    public HashMap<String, String> ports = new HashMap<>();
}
