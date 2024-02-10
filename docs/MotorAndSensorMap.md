#MotorSensorMap
## DRIVETRAIN
    ### SWERVEPODS
        - Azimuth Controller =  SparkMax    PID Position        rio
        - Thrust Controller =   TalonFX     PID Velocity        canivore
        - Encoder = CANCoder                                    canivore
    ### Atag Recog  
        - Vision unit FR =      Photonvision/OrangePi
        - Vision unit FL =      Photonvision/OrangePi
        - Vision unit BL =      Photonvision/OrangePi
        - Vision unit BR =      Photonvision/OrangePi
## INTAKE
    ### Grabber
        - Grab Controller =     TalonFX     PID Velocity or %    
    ### Pivot
        - Pivot Controller =    TalonFX     PID Position 
    ### Elevator
        - Elevator Controller1= TalonFX     PID Position
        - Elevator Controller2= TalonFX     Follower
    ### Note Recog
        - Vision unit 1 =       Photonvision/OrangePi
## Shooter
    ### Angler
        - Angler Controller =   Talon FX    PID Position
    ### Launcher
        - Launch Controller1 =  TalonFX     PID Velocity
        - Launch Controller2 =  TalonFX     PID Velocity
## Climb
    ### Climb Right
        - Right Controller =    TalonFX     PID Position
    ### Climb Left 
        - Left Controller =     TalonFX     PID Position
