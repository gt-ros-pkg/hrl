##
# Periodically logs the output of a callback function by calling it at a certain
# rate and gathering up the results into a list
class PeriodicLogger():
    ##
    # initializes the logger but doesn't start it
    #
    # @param callback the function to be called each time
    # @param rate the rate in seconds at which to call the callback
    # @param args the function arguments to pass into the callback
    def __init__(self, callback, rate=0.01, args=None):
        self.ret = []
        self.cb = callback
        self.rate = rate
        self.args = args
        self.is_running = False
        self.max_calls = None
        self.num_calls = 0
        self.beg_time = 0.
        self.thread = None

    ##
    # begins the logger 
    # @param max_calls the maximum number of times to call the callback
    def start(self, max_calls=None):
        if self.is_running:
            return
        self.max_calls = max_calls
        self.is_running = True
        self.num_calls = 0
        self.beg_time = rospy.Time.now().to_sec()
        self.thread = threading.Timer(self.rate, self._run)
        self.thread.start()

    def _run(self):
        if not self.is_running:
            return

        act_time = self.beg_time + self.rate * (self.num_calls + 2)
        interval = act_time - rospy.Time.now().to_sec()
        self.thread = threading.Timer(interval, self._run)
        self.thread.start()

        if self.args is None:
            retval = self.cb()
        else:
            retval = self.cb(*self.args)
        self.ret += [retval]

        self.num_calls += 1
        # break if we have called the sufficent number of times
        if self.max_calls is not None:
            if self.num_calls == self.max_calls:
                self.is_running = False
                return

    ##
    # stops the monitor
    # @return the result of the monitor
    def stop(self):
        self.thread.cancel()
        if not self.is_running:
            return None
        self.is_running = False
        return self.ret

    ##
    # If max_calls sets to automatically terminate, return the ret vals
    def get_ret_vals(self):
        if self.is_running:
            return None
        return self.ret

##
# Periodically monitors the output of a callback function by calling it at a certain
# rate and compares it with a provided model to insure the value doesn't vary greatly
# within a degree of tolerance provided by the variance function
class PeriodicMonitor():
    ##
    # initializes the monitor but doesn't start it
    #
    # @param callback the function to be called each time
    # @param rate the rate in seconds at which to call the callback
    # @param args the function arguments to pass into the callback
    def __init__(self, callback, rate=0.01, args=None):
        self.ret = []
        self.cb = callback
        self.rate = rate
        self.args = args
        self.is_running = False
        self.num_calls = 0
        self.beg_time = 0.
        self.thread = None
        self.mean_model = None
        self.variance_model = None
        self.std_devs = 0.
        self.failure = False

    ##
    # begins the monitor
    # TODO DOCS
    # @param max_calls the maximum number of times to call the callback
    def start(self, mean_model, variance_model, std_devs=2.5, max_calls=None, 
                                                contingency=None, contingency_args=None):
        if len(mean_model) != len(variance_model):
            log("Models must be of same length")
            return
        if self.is_running:
            return
        self.is_running = True
        self.mean_model = mean_model
        self.variance_model = variance_model
        self.std_devs = std_devs
        self.max_calls = max_calls
        self.contingency = contingency
        self.contincency_args = contingency_args
        self.model_index = 0
        self.failure = False
            
        self.num_calls = 0
        self.beg_time = rospy.Time.now().to_sec()
        self.thread = threading.Timer(self.rate, self._run)
        self.thread.start()

    def _run(self):
        if not self.is_running:
            return

        act_time = self.beg_time + self.rate * (self.num_calls + 2)
        interval = act_time - rospy.Time.now().to_sec()
        self.thread = threading.Timer(interval, self._run)
        self.thread.start()

        if self.args is None:
            retval = self.cb()
        else:
            retval = self.cb(*self.args)

        # go through each coordinate in the vector
        for coord_i in len(retval[1]):
            diff = abs(retval[1][coord_i] - self.mean_model[self.model_index][coord_i])
            deviation = np.sqrt(self.variance_model[self.model_index][coord_i])
            if diff > self.std_devs * deviation:
                # signal is outside allowable range
                self.failure = True
                self.is_running = False
                # call contingency function
                if contingency_args is None:
                    self.contingency()
                else:
                    self.contingency(*contingency_args)
                return
        self.ret += [retval]
        self.model_index += 1
        if self.model_index == len(self.mean_model):
            self.is_running = False
            return

        # break if we have called the sufficent number of times
        if not self.max_calls is None:
            self.max_calls -= 1
            if self.max_calls == 0:
                self.is_running = False
                return

    ##
    # stops the monitor
    # @return the result of the monitor
    def stop(self):
        self.thread.cancel()
        if not self.is_running:
            return None
        self.is_running = False
        return self.ret

    ##
    # If max_calls sets to automatically terminate, return the ret vals
    def get_ret_vals(self):
        if self.is_running:
            return None
        return self.ret

    # TODO DOCS
    def has_failed():
        return self.failure

    # TODO DOCS
    def wait_for_completion(rate=0.01):
        while self.is_running and not rospy.is_shutdown():
            rospy.sleep(rate)
        return not self.failure
