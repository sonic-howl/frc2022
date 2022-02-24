from typing import List

class AutoRecorder:
    def __init__(self, motors):
        self.motors: List = motors
        self.auto_record = []
        # Counter for auto-playback
        self.auto_playback_counter = 0

    def recordAuto(self):
        list = []
        for motor in self.motors:
            list.append(motor.get())
        self.auto_record.append(list)

    def playAuto(self):
        if self.auto_playback_counter < len(self.auto_record):
            motorvalues = self.auto_record[self.auto_playback_counter]
            for i in range(len(motorvalues)):
                self.motors[i].set(motorvalues[i])
            self.auto_playback_counter += 1
"""
        self.auto_record = [[0.3, 0.2, 1.0], [0.3, 0.2, 1.0], [0.3, 0.2, 1.0], [0.3, 0.2, 1.0]]
"""
