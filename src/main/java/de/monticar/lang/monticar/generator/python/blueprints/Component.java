package de.monticar.lang.monticar.generator.python.blueprints;

import de.monticar.lang.monticar.generator.python.ConversionHelper;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.math.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.generator.Helper;
import de.monticore.lang.monticar.generator.order.ImplementExecutionOrder;
import de.monticore.symboltable.Scope;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class Component {
    private final ArrayList<Port> ports;
    private final ArrayList<PortArray> portArrays;
    private final ArrayList<Connector> connectors;
    private final ArrayList<Instance> instances;
    private final HashMap<String, String> imports;
    private final List<String> parameter;
    private List<String> pythonCommands;
    private String name;
    private String fullName;
    private String type;
    private boolean hasNumpy;
    private boolean isRosPub;
    private boolean isRosSub;

    public Component(ExpandedComponentInstanceSymbol symbol, Scope symtab) {
        ports = new ArrayList<>();
        portArrays = new ArrayList<>();
        connectors = new ArrayList<>();
        instances = new ArrayList<>();
        imports = new HashMap<>();
        pythonCommands = new ArrayList<>();
        type = symbol.getComponentType().getFullName();
        name = symbol.getComponentType().getName();
        // fullName = symbol.getFullName();
        parameter = symbol.getParameters().stream()
                .map(s -> s.getName())
                .collect(Collectors.toList());
        hasNumpy = false;
        List<String> executionOrder = ImplementExecutionOrder.exOrder(symbol).stream()
                .map(s -> s.getName())
                .collect(Collectors.toList());
        symbol.getPorts().forEach(pSymbol -> addPort(pSymbol));
        symbol.getSubComponents().stream()
                .sorted((s1, s2) -> executionOrder.indexOf(s1.getName()) < executionOrder.indexOf(s2.getName()) ? -1 : 1)
                .forEach(subSymbol -> {
                    instances.add(new Instance(subSymbol, this));
                });
        symbol.getConnectors().forEach(cSymbol ->
                connectors.add(new Connector(cSymbol)));
        instances.forEach(instance -> imports.put(instance.getPath().toLowerCase(), instance.getType()));
        MathStatementsSymbol mst = Helper.getMathStatementsSymbolFor(symbol, symtab);
        if(Objects.nonNull(mst)){
            List<MathExpressionSymbol> mExpressions = Helper.getMathStatementsSymbolFor(symbol,
                    symtab).getMathExpressionSymbols();
            pythonCommands = Arrays.stream(mExpressions.stream()
                    .filter(Objects::nonNull)
                    .map(e -> addBehaviour(e))
                    .reduce("", (a, b) -> a + b)
                    .split(";")).collect(Collectors.toList());
        }

        isRosPub =
                type.equals("RosPublisher");
        isRosSub =  type.equals("RosSubscriber");

    }


    public void addPort(PortSymbol portSymbol){
        Port port = new Port(portSymbol);
        if(portSymbol.isPartOfPortArray()){
            Optional<PortArray> parent = portArrays.stream()
                    .filter(portArray -> portArray.isSameType(port))
                    .findFirst();
            if (parent.isPresent()) {
                parent.get().addPort();
            } else {
                portArrays.add(new PortArray(port));
            }
        }
        else{
            ports.add(port);
        }
    }

    private String addBehaviour(MathExpressionSymbol expression) {
        List<String> names = Stream.concat(ports.stream().map(p -> p.getName()), instances.stream().map(i -> i.getName())).collect(Collectors.toList());
        ConversionHelper.setNames(names);
        return ConversionHelper.getMathBehaviour(expression);
    }

    public ArrayList<Port> getPorts() {
        return ports;
    }

    public ArrayList<PortArray> getPortArrays() {
        return portArrays;
    }

    public ArrayList<Connector> getConnectors() {
        return connectors;
    }

    public String getFullName() {
        return fullName;
    }

    public String getName() {
        return name;
    }

    public String getType() {
        return type;
    }

    public ArrayList<Instance> getInstances() {
        return instances;
    }

    public List<String> getParameter(){
        return parameter;
    }

    public void setHasNumpy(boolean hasNumpy) {
        this.hasNumpy = hasNumpy;
    }

    public boolean getHasNumpy() {
        return hasNumpy;
    }

    public List<String> getPythonCommands() {
        return pythonCommands;
    }

    public HashMap<String, String> getImports(){
        return imports;
    }
    public boolean equals(ExpandedComponentInstanceSymbol symbol) {
        return symbol.getFullName().equals(fullName);
    }

    public boolean isRosPub() {
        return isRosPub;
    }

    public boolean isRosSub() {
        return isRosSub;
    }

    @Override
    public String toString() {
        return "Component{" +
                "ports=" + ports +
                ", portArrays=" + portArrays +
                ", connectors=" + connectors +
                ", name='" + name + '\'' +
                '}';
    }
}
