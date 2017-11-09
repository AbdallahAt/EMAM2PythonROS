[# th:if="${hasRosWrapper}"]import rospy
[# th:each="message : ${messages}"]
from components.messages import [(${message})][/][/]
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
    [# th:if="${hasRosWrapper}"]rospy.init_node()[/]
    [# th:each="pub, iter: ${tag.publisher}" ]
        self.pub[(${iter.count})] = rospy.Publisher([(${pub.topic})], [(${pub.type})])
    [/]
    [# th:each="sub, iter: ${tag.subscriber}" ]
        rospy.Subscriber([(${sub.topic})], [(${sub.type})], self.handleSub[(${iter.count})])
    [/]

    def execute(self): [# th:if="${instances.isEmpty() && commands.isEmpty() && !hasRosWrapper}"]
        pass  [/]
        [# th:each="command : ${commands}" ]
        [(${command})]
        [/]
        [# th:each="instance : ${instances}" ]
        self.[(${instance.getName()})].execute()
        [/]
        [# th:each="pub, iter : ${tag.publisher}" ]
        self.pub[(${iter.count})].publish(self.handlePub[(${iter.count})]())
        [/]
    [# th:each="sub, iter : ${tag.subscriber}" ]
    def handleSub[(${iter.count})](self, msg):
        [# th:each="port: ${sub.ports.entrySet()}" ]
        self.[(${port.getKey()})].value = msg.[(${port.getValue()})]
        [/]
    [/]
    [# th:each="pub, iter : ${tag.publisher}" ]
    def handlePub[(${iter.count})](self):
        msg = [(${pub.type})]()
    [# th:each="port: ${pub.ports.entrySet()}" ]
        msg.[(${port.getValue()})] = [(${port.getKey()})].value
        [/]
        return msg
    [/]
