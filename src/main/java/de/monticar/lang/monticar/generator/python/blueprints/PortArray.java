package de.monticar.lang.monticar.generator.python.blueprints;

public class PortArray {
    private Port port;
    private int length = 0;

    public PortArray(Port port) {
        this.port = port;
        length++;
    }

    public boolean isSameType(Port port){
        return this.port.equals(port);
    }

    public void addPort(){
        length++;
    }

    @Override
    public String toString() {
        return "PortArray{" +
                "port=" + port +
                ", length=" + length +
                '}';
    }
}
