## Crazyflie pkg
### Config folder
crazyflies.yaml
```
robots:
    cf231:
        ...
        uri: radio://0/80/2M/E7E7E7E7XX
        ...
        firmware_logging:
            enabled: true
            custom_topics:
                control: 
                frequency: 100
                vars: ["motor.m1", "motor.m2", "motor.m3", "motor.m4"]
                state:
                frequency: 100
                vars: ["stateEstimate.y",
                    "stateEstimate.z",
                    "stateEstimate.vy",
                    "stateEstimate.vz",
                    "stateEstimate.roll"]
...
all:
    firmware_params:
        stabilizer:
            controller: 1 # 1 for PID 2 for mellinger
    
```