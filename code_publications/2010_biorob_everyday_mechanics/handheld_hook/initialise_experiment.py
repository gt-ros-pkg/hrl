import exceptions


##
# get number from command line. Check if the number is of the
# appropriate dataype.
# @param dtype - int, float etc. 
def get_number(prompt, dtype):
    no_number = True
    number = -9999.0
    while no_number:
        print prompt
        str = raw_input()
        try:
            number = dtype(str)
            no_number = False
        except exceptions.ValueError, e:
            print 'That was not a valid %s! Try again.'%(dtype.__str__)

    return number


if __name__ == '__main__':
    import hrl_lib.util as hut

    checkerboard_number = get_number('Enter visible hook checkerboard number (number on top right hand corner of checkerboard)',
                                     dtype = int)
    height = get_number('Enter height of the mechanism checkerboard origin (top left hand corner).', dtype = float)
    radius = get_number('Enter distance of handle (hooking location) from the hinge of door. (-1 for drawer)', dtype = float)
    print 'Please take a picture of the mechanism. Press enter when done'
    raw_input()

    s = { 
          'checkerboard_number': checkerboard_number,
          'height': height,
          'radius': radius
        }

    hut.save_pickle(s, 'mechanism_info.pkl')


