def enable_laser(self, b):
        """ Activates or deactivates the laser depending on whether the value of b is True or False. """
        msg = ''

        if b == True:
            msg = self.__envia('SetLDSRotation On', "Enable Laser")
        else:
            msg = self.__envia('SetLDSRotation Off', "Disable Laser")

        if msg != '\032':
            print "[ROBOT LASER]", msg
def __get_laser(self):
        """ Ask to the robot for the current values of the laser. """
        msg = self.__envia('GetLDSScan', "GetLDSScan")
        
        self.laser_mutex.acquire()
        self.laser_values = []

        for line in msg.split('\r\n')[2:362]:
            s = line.split(',')
            lr = self.laser_row(s[0], s[1], s[2], s[3])
            self.laser_values.append(lr)
        
        self.laser_mutex.release()
