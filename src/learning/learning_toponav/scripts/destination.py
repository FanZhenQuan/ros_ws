#!/usr/bin/env python

import rospy


class Idleness(object):
    def __init__(self, true, remaining, estim):
        self.__true_idl = true
        self.__estim_idl = estim
        self.__remaining_idl = remaining
    
    def get_true(self):
        return self.__true_idl
    
    def get_remaining(self):
        return self.__remaining_idl

    def get_estimated(self):
        return self.__estim_idl
    
    def is_null(self):
        """
        an idleness is considered "null" if it portraits the last
        idleness before the simulation shutdown
        """
        return self.__remaining_idl == self.__true_idl and self.__estim_idl == 0
    
    def is_first(self):
        return self.__true_idl >= 60.00
    
    def get_estimate_index(self, _type=float):
        """
        :param _type: type of return value
        :return: (str or float) ratio between remaining idleness and estimated idleness
        """
        if _type == float:
            return self.__remaining_idl / self.__estim_idl
        elif _type == str:
            return "%s/%s" % (self.__remaining_idl, self.__estim_idl)
        
    def __str__(self):
        return "true: %s, rem: %s, est: %s" % (self.__true_idl, self.__remaining_idl, self.__estim_idl)
    
    def __repr__(self):
        return "T: %s, R: %s, E: %s" % (self.__true_idl, self.__remaining_idl, self.__estim_idl)
    

class Observation(object):
    def __init__(self, idleness, path_len):
        if not isinstance(idleness, Idleness):
            raise TypeError("Observation could not be created: %s not of type Idleness" % idleness)
        if not isinstance(path_len, float):
            try:
                float(path_len)
            except:
                raise TypeError("Observation could not be created: %s not of type float" % path_len)
        
        self.idleness = idleness
        self.path_len = path_len
        
    def get_interference(self):
        interf = self.idleness.get_remaining() - self.idleness.get_estimated()
        
        return interf / self.path_len
    
    def is_null(self):
        return self.idleness.is_null()
    
    def is_first(self):
        return self.idleness.is_first()
    
    def __repr__(self):
        return "(%s), %s" % (self.idleness.__repr__(), self.path_len)


class Destination(object):
    def __init__(self, name, pose, available=True):
        self.name = name
        self.pose = pose
        self.estim_idl = 0
        self.__latest_usage = rospy.Time.now()
        self.__remaining_idl = rospy.Time.now()
        self.__stats = []  # stores every idleness registered
        self.__available = available
        
        self.path_len = 0.0  # path len from robot pos to this dest
        
    @property
    def available(self):
        return self.__available
    
    @available.setter
    def available(self, value):
        latest_available = self.__available
        self.__available = value
        if(
            latest_available is False and
            self.__available is True
        ):  # destination has been freed, reset idleness
            self.__append_idleness()
            self.reset()
        elif(
            latest_available is True and
            self.__available is False
        ):  # destination has been chosen, set ac_idleness
            self.__remaining_idl = rospy.Time.now()
        
    def get_true_idleness(self):
        """
        :return: seconds elapsed since latest usage (type: float)
        """
        now = rospy.Time.now()
        return (now - self.__latest_usage).to_sec()
        
    def __append_idleness(self):
        true_idl = self.get_true_idleness()
        
        if true_idl >= 0:
            self.__stats.append(
                Observation(
                    Idleness(
                        true=true_idl,
                        estim=self.estim_idl,
                        remaining=(rospy.Time.now() - self.__remaining_idl).to_sec()
                    ),
                    self.path_len
                )
            )
        
    def get_stats(self):
        return self.__stats
    
    def get_visits(self):
        """
        a visit is an idleness which is not null, meaning that
        true_idl != remaining_idl and estim_idl != 0
        :return: (list) visits
        """
        # if len(self.__stats) == 1:
        #     return 0
        # else:
        #     count = 0
        #     for observ in self.__stats:
        #         if not observ.idleness.is_null():
        #             count += 1
        #
        #     return count
        visits = []
        for observ in self.__stats:
            if not observ.is_null() and not observ.is_first():
                visits.append(observ)
                
        return visits
        
    def reset(self):
        self.estim_idl = 0
        self.path_len = 0.0
        self.__latest_usage = rospy.Time.now()
        self.__remaining_idl = rospy.Time.now()
        
    def force_shutdown(self):
        self.__append_idleness()
        self.reset()
        
    def __str__(self):
        return self.name
    
    def __repr__(self):
        return self.name
    
    def __eq__(self, other):
        if isinstance(other, Destination):
            return self.name == other.name
        else:
            raise TypeError('%s is not of type <Destination>' % other)
