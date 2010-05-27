import hrl_lib.rutils as ru
from sensor_msgs.msg import Image

topics = ["/wide_stereo/left/image_color", "/wide_stereo/right/image_color"]
listener = ru.GenericListener('stereo_listener', [Image, Image], topics, 30.0)

while True:
    r = listener.read(allow_duplication=False, willing_to_wait=True, warn=False, quiet=True)
    print r[0].__class__, r[1].__class__

