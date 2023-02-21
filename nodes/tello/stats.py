from datetime import datetime

## This Class provides a method of saving the responses of commands with their respective commands.
# 
class Stats:
    ## Creates a new instance of this class with a command string and the incremented id. Begins a timer for the ping time.
    #  @param self The object pointer.
    #  @param command The command sent as a string.
    #  @param id The increasing id of the command sent.
    def __init__(self, command, id):
        self.command = command
        self.response = None
        self.id = id

        self.start_time = datetime.now()
        self.end_time = None
        self.duration = None

    ## Updates the response of the command to this command. Ends the timer and sets the response time.
    #  @param self The object pointer.
    #  @param response The object pointer.
    def add_response(self, response):
        self.response = str(response)
        # Calculating total time taken to execute command
        self.end_time = datetime.now()
        self.duration = (self.end_time-self.start_time).total_seconds()

    ## Returns whether the reponse was received or not
    #  @param self The object pointer.
    def got_response(self):
        if self.response is None:
            return False
        else:
            return True

    ## Returns the raw response string.
    #  @param self The object pointer.
    def get_raw_response(self):
        return self.response

    ## Returns the numeric response string.
    #  @param self The object pointer.
    #  @param data The data to convert.
    def numeric_response(self, data):
        num_val = ''.join(i for i in data if i.isdigit() or i=='-' or i=='.')
        return num_val
    
    ## Returns the reponse as an int
    #  @param self The object pointer.
    #  @param data The data to convert.
    def int_response(self, data):
        return int(self.numeric_response(data))

    ## Returns the reponse as a float
    #  @param self The object pointer.
    #  @param data The data to convert.
    def float_response(self, data):
        return float(self.numeric_response(data))
    
    ## Returns the attitude as an int triple
    #  @param self The object pointer.
    def attitude_response(self):
        raw_att = self.response.split(';')
        att_data = (self.int_response(raw_att[0]), self.int_response(raw_att[1]), self.int_response(raw_att[2]))
        return att_data
    
    ## Returns the acceleration as a float triple
    #  @param self The object pointer.
    def acceleration_response(self):
        raw_acc = self.response.split(';')
        acc_data = (self.float_response(raw_acc[0]), self.float_response(raw_acc[1]), self.float_response(raw_acc[2]))
        return acc_data

    ## Returns the temperature as an int couple
    #  @param self The object pointer.
    def temp_response(self):
        raw_temp = self.response.split('~')
        temp = (self.int_response(raw_temp[0]) + self.int_response(raw_temp[1]))/2
        return temp

    ## Returns the reponse of the given command
    #  @param self The object pointer.
    def get_response(self):
        if self.response is None:
            return None
        elif self.response is '':
            return ''
        elif 'attitude?' in self.command:
            return self.attitude_response()
        elif 'acceleration?' in self.command:
            return self.acceleration_response()
        elif 'temp?' in self.command:
            return self.temp_response()
        elif 'baro?' in self.command or 'speed?' in self.command:
            return self.float_response(self.response)
        elif '?' not in self.command:
            return self.get_raw_response()
        else:
            return self.int_response(self.response)