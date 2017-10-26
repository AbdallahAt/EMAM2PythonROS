package de.monticar.lang.monticar.generator.python;

import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixExpressionSymbol;
import java.util.ArrayList;
import java.util.List;

public class ConversionHelper {
    private static List<String> names;

    public static String mathSymbolToNumpy(String mathText) {
        ArrayList<ArrayList<String>> matrix = getMatrix(mathText.toCharArray());
        String rows = String.valueOf(matrix.toArray().length);
        String cols = String.valueOf(matrix.get(0).toArray().length);
        String numpyCommand = matrix.stream()
                .flatMap(ArrayList::stream)
                .reduce("np.array([", (a, b) -> a + b + ",");
        numpyCommand = numpyCommand.substring(0, numpyCommand.length() - 1);
        numpyCommand += "]).reshape(" + rows + "," + cols + ")";
        return numpyCommand;
    }

    public static ArrayList<ArrayList<String>> getMatrix(char[] arr){
        ArrayList<ArrayList<String>> matrix = new ArrayList<>();
        matrix.add(new ArrayList<>());
        String curNumber = "";
        int activeRow = 0;
        for (char c : arr) {
            if ((c == ' ' || c == ';' || c == ')' || c == ',') && curNumber.length() > 0) {
                matrix.get(activeRow).add(curNumber);
                curNumber = "";
                if (c == ';') {
                    activeRow++;
                    matrix.add(new ArrayList<>());
                }
            }
            else if(Character.isDigit(c)){
                curNumber += c;
            }
        }

        return matrix;
    }

    public static MathExpressionPython getExpressionType(MathExpressionSymbol expression){
        if(expression.isForLoopExpression()){
            return MathExpressionPython.FOR;
        }

        if(expression.isAssignmentExpression()){
            return MathExpressionPython.ASSIGNMENT;
        }

        if(expression.isArithmeticExpression()){
            return MathExpressionPython.ARITHMETIC;
        }

        if(expression.isValueExpression()){
            if (isPortName(expression.getTextualRepresentation())) return MathExpressionPython.PORT;
            return MathExpressionPython.VALUE;
        }

        if(expression.isMatrixExpression()){
            MathMatrixExpressionSymbol mat = (MathMatrixExpressionSymbol) expression;
            if(mat.isMatrixNameExpression()) return MathExpressionPython.MATRIX;
        }

        if(expression.isConditionalsExpression()){
            return MathExpressionPython.CONDITIONAL;
        }

        if(expression.isCompareExpression()){
            return MathExpressionPython.COMPARE;
        }
        return MathExpressionPython.UNKNOWN;
    }

    public static Boolean isPortName(String name){
        if (names.contains(name)) return true;
        return false;
    }
    public static String getMathBehaviour(MathExpressionSymbol expression){
        return getExpressionType(expression).getBehaviour(expression);
    }

    public static void setNames(List<String> names) {
        ConversionHelper.names = names;
    }
}
