package de.monticar.lang.monticar.generator.python.blueprints;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;

public class Connector {
    private String source;
    private String target;

    public Connector(ConnectorSymbol symbol) {

        this.source = symbol.getSource();
        this.target = symbol.getTarget();
    }

    public String getSource(){
        return source;
    }

    public String getTarget(){
        return target;
    }
}
