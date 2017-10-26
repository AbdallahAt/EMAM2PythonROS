package de.monticar.lang.monticar.generator.python.blueprints;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConstantPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;

import java.util.HashMap;

public class Port {
    private String type = "None";
    private String name = "";
    private String value = "";
    static final HashMap<String, String> type2Value;

    static{
        type2Value = new HashMap<>();
        type2Value.put("SIUnitRangesType", "0");
        type2Value.put("Boolean", "False");
        type2Value.put("ElementType", "0");
    }
    public Port(PortSymbol port) {
        name = port.getNameWithoutArrayBracketPart();
        type = port.getTypeReference().getName();
        value = port.isConstant() ?
                ((ConstantPortSymbol) port).getConstantValue().getValueAsString() : type2Value.getOrDefault(type, "None");

    }

    public String getType() {
        return type;
    }

    public String getName() {
        return name;
    }

    public String getValue() {
        return value;
    }

    @Override
    public boolean equals(Object o){
        return o instanceof Port &&
                ((Port) o).getName().equals(name) &&
                ((Port) o).getType().equals(type);
    }

    @Override
    public String toString() {
        return "Port{" +
                "type='" + type + '\'' +
                ", name='" + name + '\'' +
                '}';
    }
}
