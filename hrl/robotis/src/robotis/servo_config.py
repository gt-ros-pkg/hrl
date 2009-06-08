
# home position in encoder ticks for the servo.
# to add - angle limits?

servo_param = {
    1: {                        # Default for new servo.  Please issue 'new_servo.write_id(new_id)' and setup your own home position!
        'home_encoder': 351
       }, 
    2: {                        # Servo for the Tilting Hokuyo
        'home_encoder': 446
       }
}

