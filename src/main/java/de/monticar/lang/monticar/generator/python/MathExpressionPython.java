package de.monticar.lang.monticar.generator.python;

import de.monticore.lang.math.math._symboltable.expression.*;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public enum MathExpressionPython {
    FOR {
        @Override
        public String getBehaviour(MathExpressionSymbol expression){
            MathForLoopExpressionSymbol forExpression = (MathForLoopExpressionSymbol) expression;
            MathMatrixVectorExpressionSymbol forHead = (MathMatrixVectorExpressionSymbol) forExpression.getForLoopHead().getMathExpression();
            String forHeadPython = transformForHead(forHead);
            List<String> forBody = forExpression.getForLoopBody().stream()
                    .map(e -> ConversionHelper.getMathBehaviour(e))
                    .map(e -> "    " + e)
                    .collect(Collectors.toList());

            forBody.add(0, forHeadPython);
            return forBody.stream().collect(Collectors.joining(";"));

        }

        private String transformForHead(MathMatrixVectorExpressionSymbol forHead){
            String start = forHead.getStart().getTextualRepresentation() ;

            Optional<MathExpressionSymbol> stepExpression= forHead.getStep();
            String step = stepExpression.isPresent() ? stepExpression.get().getTextualRepresentation() : "1";

            String end = forHead.getEnd().getTextualRepresentation();

            return "for i in range(" + start + ", " + step + ", " + end + "): ";
        }
    },

    ASSIGNMENT {
        @Override
        public String getBehaviour(MathExpressionSymbol expression) {
            MathAssignmentExpressionSymbol assignment = (MathAssignmentExpressionSymbol) expression;
            String lValuePython = "self." + assignment.getNameOfMathValue();
            lValuePython += ConversionHelper.isPortName(assignment.getNameOfMathValue()) ? ".value" : "";
            String operatorPython = assignment.getAssignmentOperator().getOperator();
            String rValuePython = ConversionHelper.getMathBehaviour(assignment.getExpressionSymbol());
            return lValuePython + operatorPython + rValuePython + ";";
        }
    },

    ARITHMETIC {
        @Override
        public String getBehaviour(MathExpressionSymbol expression) {
            MathArithmeticExpressionSymbol arithmetic = (MathArithmeticExpressionSymbol) expression;
            String operator = arithmetic.getMathOperator();
            String leftExpressionPython = ConversionHelper.getMathBehaviour(arithmetic.getLeftExpression());
            String rightExpressionPython = ConversionHelper.getMathBehaviour(arithmetic.getRightExpression());
            return leftExpressionPython + operator + rightExpressionPython;
        }
    },

    CONDITIONAL{
        @Override
        public String getBehaviour(MathExpressionSymbol expression){
            MathConditionalExpressionsSymbol cond = (MathConditionalExpressionsSymbol) expression;
            MathConditionalExpressionSymbol ifCond = cond.getIfConditionalExpression();
            Optional<MathConditionalExpressionSymbol> elseCond = cond.getElseConditionalExpression();
            String ifCondPython = "";
            List<String> ifBody = new ArrayList<>();
            List<String> elseBody = new ArrayList<>();
            ifCondPython = ConversionHelper.getMathBehaviour(ifCond.getCondition().get());
            ifBody = getBody(ifCond.getBodyExpressions());
            if(elseCond.isPresent()){
                elseBody = getBody(elseCond.get().getBodyExpressions());
                elseBody.add(0, "else: ");
            }
            ifBody.add(0, "if (" + ifCondPython + "): ");
            return Stream
                    .concat(ifBody.stream(), elseBody.stream())
                    .collect(Collectors.joining(";"));
        }

        public List<String> getBody(List<MathExpressionSymbol> expression){
            return expression.stream()
                    .map(e -> ConversionHelper.getMathBehaviour(e))
                    .map(e -> "    " + e)
                    .collect(Collectors.toList());
        }
    },

    COMPARE{
        @Override
        public String getBehaviour(MathExpressionSymbol expression) {
            MathCompareExpressionSymbol comp = (MathCompareExpressionSymbol) expression;
            String operator = comp.getCompareOperator();
            String leftExpressionPython = ConversionHelper.getMathBehaviour(comp.getLeftExpression());
            String rightExpressionPython = ConversionHelper.getMathBehaviour(comp.getRightExpression());
            return leftExpressionPython + operator + rightExpressionPython;
        }
    },

    MATRIX {
        @Override
        public String getBehaviour(MathExpressionSymbol expression) {
            return ConversionHelper.mathSymbolToNumpy(expression.getTextualRepresentation());
        }
    },

    VALUE {
        @Override
        public String getBehaviour(MathExpressionSymbol expression){
            MathValueExpressionSymbol value = (MathValueExpressionSymbol) expression;
            if (value.isNumberExpression()) return value.getTextualRepresentation();
            else if (expression.getName().equals(""))  return "self." + value.getTextualRepresentation();
            return expression.getName();
        }
    },

    UNKNOWN {
        @Override
        public String getBehaviour(MathExpressionSymbol expression){
            return "";
        }
    },

    PORT{
        @Override
        public String getBehaviour(MathExpressionSymbol expression){
            return "self." + expression.getTextualRepresentation() + ".value";
        }
    };

    public abstract String getBehaviour(MathExpressionSymbol expression);
}
