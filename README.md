# Obstacle_Avoidance

## UML Component diagram(raw)
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