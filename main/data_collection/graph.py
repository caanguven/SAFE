flowchart TD
    subgraph Client ["Client Device (Browser)"]
        UI[Web Interface]
        Video[Video Display]
        IMU[IMU Visualization]
        Controls[Control Interface]
    end

    subgraph Network ["WiFi Network"]
        HTTP[HTTP/Websocket Communication]
    end

    subgraph RaspberryPi ["Raspberry Pi (Flask Server)"]
        subgraph FlaskApp ["Flask Application"]
            Server[Flask Server]
            VideoStream[Video Streaming]
            IMUProcess[IMU Processing]
            MotorControl[Motor Control System]
        end
        
        subgraph Hardware ["Hardware"]
            Camera[PiCamera2]
            IMUSensor[BNO08X IMU]
            Motors[Servo Motors]
        end
    end

    %% Client to Server Communications
    UI --> |HTTP Request| Server
    Video --> |MJPEG Stream| VideoStream
    IMU --> |Periodic Updates| IMUProcess
    Controls --> |POST Requests| MotorControl

    %% Internal Server Communications
    Server --> VideoStream
    Server --> IMUProcess
    Server --> MotorControl

    %% Hardware Connections
    Camera --> VideoStream
    IMUSensor --> IMUProcess
    MotorControl --> Motors

    %% Network Connections
    HTTP --- |Port 5000| Server

    classDef clientNode fill:#a5d8ff,stroke:#0066cc,stroke-width:2px;
    classDef serverNode fill:#d4edda,stroke:#28a745,stroke-width:2px;
    classDef hardwareNode fill:#fff3cd,stroke:#ffc107,stroke-width:2px;
    classDef networkNode fill:#f8d7da,stroke:#dc3545,stroke-width:2px;

    class UI,Video,IMU,Controls clientNode;
    class Server,VideoStream,IMUProcess,MotorControl serverNode;
    class Camera,IMUSensor,Motors hardwareNode;
    class HTTP networkNode;