[# th:if="${hasNumpy}"]import numpy as np[/]
from components.common.port import Port
[# th:each="import : ${imports.entrySet()}" ]
from components.[(${import.getKey()})] import [(${import.getValue()})]
[/]

class [(${name})]:
    def __init__(self[# th:unless="${parameter.isEmpty()}"],[/][# th:each="entry, iter: ${parameter}"] [(${entry})][# th:unless="${iter.last}"], [/][/]):
    [# th:each="port : ${ports}" ]
        self.[(${port.getName()})] = Port([(${port.getValue()})])
    [/] [# th:each="instance : ${instances}" ]
        self.[(${instance.getName()})] = [(${instance.getType()})]([# th:each="argument, iter : ${instance.getArguments()}"][(${argument})][# th:unless="${iter.last}"], [/][/])
    [/]
    [# th:each="connector : ${connectors}" ]
        self.[(${connector.getTarget()})] = self.[(${connector.getSource()})]
    [/]
    def execute(self): [# th:if="${instances.isEmpty() && commands.isEmpty()}"]
        pass  [/]
        [# th:each="command : ${commands}" ]
        [(${command})]
        [/]
        [# th:each="instance : ${instances}" ]
        self.[(${instance.getName()})].execute()
        [/]

