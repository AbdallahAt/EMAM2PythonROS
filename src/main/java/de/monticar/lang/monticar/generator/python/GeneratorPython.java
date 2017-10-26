package de.monticar.lang.monticar.generator.python;


import de.monticar.lang.monticar.generator.python.blueprints.Component;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.symboltable.Scope;
import org.apache.commons.io.FileUtils;
import org.thymeleaf.TemplateEngine;
import org.thymeleaf.context.Context;
import org.thymeleaf.templatemode.TemplateMode;
import org.thymeleaf.templateresolver.ClassLoaderTemplateResolver;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class GeneratorPython{
    static final ArrayList<Component> componentList = new ArrayList<>() ;
    private TemplateEngine templateEngine;
    private String generationPath;

    public GeneratorPython(String generationPath) {
        this.generationPath = generationPath;
        templateEngine = new TemplateEngine();
        ClassLoaderTemplateResolver templateResolver = new ClassLoaderTemplateResolver();
        templateResolver.setPrefix("/templates/python/");
        templateResolver.setSuffix(".py");
        templateResolver.setTemplateMode(TemplateMode.TEXT);
        templateResolver.setCharacterEncoding("UTF8");
        templateResolver.setCheckExistence(true);
        templateResolver.setCacheable(false);
        templateEngine.addTemplateResolver(templateResolver);
    }

    public GeneratorPython() {
        this("./target/generated-sources-python/" + LocalDateTime.now().toString());
    }

    public static boolean isInComponentList(ExpandedComponentInstanceSymbol symbol){
        return componentList.stream()
                .filter(component -> {
                    return component.getType().equals(symbol.getComponentType().getFullName());
                })
                .collect(Collectors.toList())
                .isEmpty();
    };

    public static Stream<ExpandedComponentInstanceSymbol> flattenComponent(ExpandedComponentInstanceSymbol symbol){
        return Stream.concat(
                Stream.of(symbol),
                symbol.getSubComponents().stream().flatMap(GeneratorPython::flattenComponent));

    }
    public void addComponent(ExpandedComponentInstanceSymbol componentSymbol, Scope symtab){
        flattenComponent(componentSymbol)
                .filter(GeneratorPython::isInComponentList)
                .map(cSymbol -> new Component(cSymbol, symtab))
                .forEach(component -> componentList.add(component));
    }

    public static ArrayList<Component> getComponentList() {
        return componentList;
    }

    public void generate(){
        File f = new File(generationPath + "/components/common");
        f.mkdirs();
        File commonDir = new File(this.getClass().getClassLoader().getResource("templates/python/common").getPath());
        componentList.forEach((Component component) -> {
            String componentPath = component.getType().toLowerCase().replace(".", "/");
            String componentPython = "";
            if(component.isRosPub()){
                try {
                    componentPython = FileUtils.readFileToString(new File(commonDir.getParent() + "/ros_publisher.py"));
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            else if (component.isRosSub()){
                try {
                    componentPython = FileUtils.readFileToString(new File(commonDir.getParent() + "/ros_subscriber.py"));
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

            else {
                Context context = new Context();
                context.setVariable("name", component.getName());
                context.setVariable("parameter", component.getParameter());
                context.setVariable("hasNumpy", component.getHasNumpy());
                context.setVariable("ports", component.getPorts().toArray());
                context.setVariable("instances", component.getInstances());
                context.setVariable("connectors", component.getConnectors());
                context.setVariable("imports", component.getImports());
                context.setVariable("empty", component.getInstances().isEmpty());
                context.setVariable("commands", component.getPythonCommands());
                componentPython = templateEngine.process("component", context);
                componentPath = component.getType().toLowerCase().replace(".", "/");
            }
            File c = new File(Paths.get(f.getParent(), componentPath).toString() + ".py");
            try {
                FileUtils.writeStringToFile(c, componentPython);
            } catch (IOException e) {
                e.printStackTrace();
            }
        });

        try {

            FileUtils.copyDirectory(commonDir, f);
            generateInitFiles(f.getParentFile());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void generateInitFiles(File target) throws IOException {
        if(target.isDirectory()){
            for (File file: target.listFiles()){
                if (file.isDirectory()){
                    generateInitFiles(file);
                }
            }
        }
        new File(target.getPath() + "/__init__.py").createNewFile();
    }
}
