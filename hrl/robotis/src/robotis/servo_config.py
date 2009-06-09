
# home position in encoder ticks for the servo.
# to add - angle limits?

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
    5: {                        # Tilting hokuyo on Firenze
        'home_encoder': 338
       },
    # 11 - 25 reserved!  (by Travis)
    11: {                        # Pan Antenna on RFID Benchtop
        'home_encoder': 506
       },
    12: {                        # Tilt Antenna on RFID Benchtop
        'home_encoder': 524
       },
    13: {                        # Pan Antenna on RFID El-E Right
        'home_encoder': 510
       },
    14: {                        # Tilt Antenna on RFID El-E Right
        'home_encoder': 504
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
    18: {                        # Ear Flap on RFID El-E Left
        'home_encoder': 508
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
        }
}

