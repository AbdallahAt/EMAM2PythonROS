package de.monticar.lang.monticar.generator.python.blueprints;

import de.monticar.lang.monticar.generator.python.ConversionHelper;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.mcexpressions._ast.ASTExpression;
import java.util.ArrayList;

public class Instance {
    private ArrayList<String> arguments;
    private String name;
    private String type;
    private String path;
    private final Component component;

    public Instance(ExpandedComponentInstanceSymbol instance, Component component) {
        arguments = new ArrayList<>();
        name = instance.getName();
        type = instance.getComponentType().getName();
        path = instance.getComponentType().getFullName();
        this.component = component;
        instance.getArguments().forEach(argument -> addArgument(argument));
    }

    public void addArgument(ASTExpression argument){
        if (argument.getSymbol().isPresent()) {
            MathExpressionSymbol symbol = (MathExpressionSymbol) argument.getSymbol().get();
            if (symbol.isMatrixExpression()) {
                this.component.setHasNumpy(true);
                arguments.add(ConversionHelper.mathSymbolToNumpy(symbol.getTextualRepresentation()));
            }
                else arguments.add(symbol.getTextualRepresentation());
        } else {
            arguments.add(argument.toString());
        }
    }

    public String getName() {
        return name;
    }

    public String getPath() {
        return path;
    }

    public String getType() {
        return type;
    }

    public ArrayList<String> getArguments() {
        return arguments;
    }

}
