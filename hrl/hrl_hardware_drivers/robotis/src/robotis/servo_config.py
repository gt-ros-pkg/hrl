import math

# home position in encoder ticks for the servo.

servo_param = {
    1: {                        # Default for new servo.  Please issue 'new_servo.write_id(new_id)' and setup your own home position!
        'home_encoder': 351
       }, 
    2: {                        # Tilting Hokuyo on El-E
        'home_encoder': 446
       }, 
    3: {                        # RFID Antenna Left Tilt
        'home_encoder': 377
       },
    4: {                        # RFID Antenna Right Tilt
        'home_encoder': 330
       },
    5: {                        # Tilting kinect on Cody
        'home_encoder': 447,
        'max_ang': math.radians(55.),
        'min_ang': math.radians(-80.)
       },
    6: {                        # EL-E stereohead Pan
        'home_encoder': 500,
        'max_ang': math.radians(90.),
        'min_ang': math.radians(-90.)
       },
    7: {                        # EL-E safetyscreen tilt.
        'home_encoder': 373
       },
    11: {                        # RFID Left Pan
        'home_encoder': 430,
        'max_ang': math.radians( 141.0 ),
        'min_ang': math.radians( -31.0 )
       },
    12: {                        # RFID Left Tilt
        'home_encoder': 507,
        'max_ang': math.radians( 46.0 ),
        'min_ang': math.radians( -36.0 )        
       },
    13: {                        # RFID Right Pan
        'home_encoder': 583,
        'max_ang': math.radians( 31.0 ),
        'min_ang': math.radians( -141.0 )        
       },
    14: {                        # RFID Right Tilt
        'home_encoder': 504,
        'max_ang': math.radians( 46.0 ),
        'min_ang': math.radians( -36.0 )        
       },
    15: {                        # Ear Flap on RFID El-E Right
        'home_encoder': 498
       },
    16: {                        # Pan Antenna on RFID El-E Left
        'home_encoder': 365
       },
    17: {                        # Tilt Antenna on RFID El-E Left
        'home_encoder': 504
       },
    18: {                        # EL-E stereohead Tilt
        'home_encoder': 495,
        'max_ang': math.radians(60.),
        'min_ang': math.radians(-20.)
       },
    19: {                        # Desktop System UTM
        'home_encoder': 633,
        'flipped': True
        },
    20: {                        # Desktop System Tilt
        'home_encoder': 381
        },
    21: {                        # Desktop System Pan
        'home_encoder': 589
        },
    22: {                        # Desktop System Roll
        'home_encoder': 454
        },
    23: {                        # Dinah Top
        'home_encoder': 379
        },
    24: {                        # Dinah Bottom
        'home_encoder': 365
        },
    25: {                        # Meka top Pan
        'home_encoder': 500
        },
    26: {                        # Meka top Tilt
        'home_encoder': 400
        },
    27: {                        # PR2 RFID Right Pan
        'home_encoder': 512
        },
    28: {                        # PR2 RFID Right Tilt
        'home_encoder': 506
        },
    29: {                        # PR2 RFID Left Pan
        'home_encoder': 544
        },
    30: {                        # PR2 RFID Left Tilt
        'home_encoder': 500
        },
    31: {                        # Playpen open/close
        'home_encoder': 381
        },
    32: {                        # Conveyor for playpen
        'home_encoder': 1
        }
}

