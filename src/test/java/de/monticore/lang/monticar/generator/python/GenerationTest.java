package de.monticore.lang.monticar.generator.python;



import de.monticar.lang.monticar.generator.python.ConversionHelper;
import de.monticar.lang.monticar.generator.python.GeneratorPython;
import de.monticar.lang.monticar.generator.python.blueprints.Component;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import java.util.ArrayList;

import static junit.framework.Assert.assertEquals;
import static junit.framework.TestCase.assertNotNull;

public class GenerationTest extends AbstractSymtabTest {
    @Test
    public void general() {
        Scope symtab = createSymTab("src/test/resources");
        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.basicPortsLoop", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorPython generatorPython = new GeneratorPython("./target/generated-sources-python/Test");
        generatorPython.addComponent(componentSymbol, symtab);
        generatorPython.generate();
    }


    @Test
    public void testBehaviour() {
        Scope symtab = createSymTab("src/test/resources");
        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("ba.delay", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorPython generatorPython = new GeneratorPython("./target/generated-sources-python/Test");
        generatorPython.addComponent(componentSymbol, symtab);
        generatorPython.generate();
    }
    @Test
    public void testDuplicateComponents() {
        Scope symtab = createSymTab("src/test/resources");
        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.lookUpInstance", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        ExpandedComponentInstanceSymbol lookUp = symtab.<ExpandedComponentInstanceSymbol>resolve("ba.lookUp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        assertNotNull(lookUp);
        GeneratorPython generatorPython = new GeneratorPython();
        generatorPython.addComponent(componentSymbol, symtab);
        generatorPython.addComponent(lookUp, symtab);
        ArrayList<Component> componentList = generatorPython.getComponentList();
        assertNotNull(componentList);
        assertEquals(componentList.toArray().length, 4);
    }

    @Test
    public void testMatrixGeneration() {
        String mat1 = "[(12 222 3333; 4 55 6; 7 8 9)]";
        String mat2 = "[(12,222, 3333; 4,55,6; 7 , 8, 9)]";

        ArrayList<ArrayList<String>> resMat1 = ConversionHelper.getMatrix(mat1.toCharArray());
        ArrayList<ArrayList<String>> resMat2 = ConversionHelper.getMatrix(mat2.toCharArray());

        ArrayList<ArrayList<String>> resMat3 = new ArrayList<ArrayList<String>>();
        ArrayList<String> row1 = new ArrayList<>();
        ArrayList<String> row2 = new ArrayList<>();
        ArrayList<String> row3 = new ArrayList<>();

        row1.add("12");
        row1.add("222");
        row1.add("3333");
        row2.add("4");
        row2.add("55");
        row2.add("6");
        row3.add("7");
        row3.add("8");
        row3.add("9");

        resMat3.add(row1);
        resMat3.add(row2);
        resMat3.add(row3);

        assertEquals(resMat1, resMat3);
        assertEquals(resMat2, resMat3);
    }

    @Test
    public void testNumpyCommandGeneration() {
        String mat1 = "[(12 222 3333; 4 55 6; 7 8 9)]";
        String res = ConversionHelper.mathSymbolToNumpy(mat1);
        assertEquals(res, "np.array([12,222,3333,4,55,6,7,8,9]).reshape(3,3)");
    }


}
