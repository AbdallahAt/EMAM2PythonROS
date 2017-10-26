/*
 * Copyright (c) 2015 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package de.monticore.lang.monticar.generator;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.java.lang.JavaDSLLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import static org.junit.Assert.assertTrue;

/**
 * Common methods for symboltable tests
 */
public class AbstractSymtabTest extends AbstractSymtab {
    public static void testFilesAreEqual(List<File> files, String restPath) {
        for (File f : files) {
            File fileTarget = new File("./src/test/resources/results/" + restPath + f.getName());
            Log.info("" + fileTarget.exists(), "Exists:");
            Log.info(f.getName() + " " + fileTarget.getName(), "Comparing:");
            assertTrue(areBothFilesEqual(f, fileTarget));
        }
    }

    public static boolean areBothFilesEqual(File file1, File file2) {
        try {
            return FileUtils.contentEquals(file1, file2);
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
    }
}
