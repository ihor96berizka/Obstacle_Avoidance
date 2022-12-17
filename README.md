# Obstacle_Avoidance

## UML Component diagram(raw)

![alternative text](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://github.com/ihor96berizka/Obstacle_Avoidance/tree/ros_compliant#readme)

@startuml


component "MainSWC" {
  portout CalculateAngle
  [Solver]
  [MainApp]
}

component "ROS" {
  portout SendLidarData
  [Sensor]
  [Actuator]
}

Sensor --> SendLidarData
SendLidarData --> MainApp

MainApp --> CalculateAngle
CalculateAngle --> Actuator
@enduml