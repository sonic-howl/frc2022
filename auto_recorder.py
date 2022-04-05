from typing import List
from threading import Thread

class AutoRecorder:
    def __init__(self, motors, period):
        # calculate iterations for auto 15 secs depending on period (ms)
        self.period = period
        ms = 1000 * self.period
        hz = 1000 / ms
        secs = 15
        self.iterations = int(hz * secs)

        self.motors: List = motors
        self.auto_record = []
        self.auto_saved = False

        # Counter for auto-playback
        self.auto_playback_counter = 0
        # Counter for auto-recording
        self.auto_recording_counter = 0

    def recordAuto(self):
        if self.auto_recording_counter < self.iterations:         
            list = []
            # for each motor in the list of motors, record the motor's speed.
            for motor in self.motors:
                list.append(motor.get())
            # append the motor speeds to the auto recording
            self.auto_record.append(list)
            self.auto_recording_counter += 1
        elif not self.auto_saved:
            self.saveAuto()
            self.auto_saved = True

            
    def playAuto(self):
        if self.auto_playback_counter <= len(self.auto_record):
            motorvalues = self.auto_record[self.auto_playback_counter]
            for i in range(len(motorvalues)):
                self.motors[i].set(motorvalues[i])
            self.auto_playback_counter += 1
        else:
            # stop all motors after the recording is done
            for motor in self.motors:
                motor.set(0)
        

    def loadAuto(self):
        def load():
            self.auto_record = []
            with open("/home/lvuser/auto_record.csv", "r") as file:
                for i in range(self.iterations):
                    line = file.readline()
                    print("LINE", line)
                    line.rstrip(", \r\n")
                    # line.rstrip(", \n")
                    # print(line)
                    motorSpeeds = line.split(", ")
                    for motorSpeed in motorSpeeds:
                        motorSpeed = float(motorSpeed)
                    motorSpeeds = motorSpeeds[:-1]
                    self.auto_record.append(motorSpeeds)
            print(self.auto_record)
        Thread(target=load).start()
            
    
    def saveAuto(self):
        def save():
            with open("/home/lvuser/auto_record.csv", "w") as file:
                for motorSpeeds in self.auto_record:
                    # motorSpeeds ex: [[0.3, 0.2, 1.0], [0.3, 0.2, 1.0], [0.3, 0.2, 1.0], [0.3, 0.2, 1.0]]
                    for motorSpeed in motorSpeeds:
                        # motorSpeed ex: [0.3, 0.2, 1.0]
                        file.write(str(motorSpeed) + ", ")
                    file.write("\n")
        Thread(target=save).start()
        
"""
"0.3, 0.2, 1.0"
        self.auto_record = [[0.3, 0.2, 1.0], [0.3, 0.2, 1.0], [0.3, 0.2, 1.0], [0.3, 0.2, 1.0]]
"""