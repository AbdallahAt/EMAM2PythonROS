import datetime
import csv
import logging

pidLogger = logging.getLogger('pid_logger')


def log_data_csv(fs, p, i, d, set_point, error, out, time):
    writer = csv.writer(fs, delimiter=',')
    writer.writerow([set_point, p, i, d, error, time, out])


class PID:
    def __init__(self, p, i, d, windup, setPoint):
        self.p = p
        self.i = i
        self.d = d
        self.windup = windup
        self.setPoint = setPoint
        self.input = 0

        self.output = 0

        self.lastError = 0
        self.totalError = 0
        self.lastTime = datetime.datetime.now()
        pidLogger.info(f"p: ${p}, i: ${i}, d: ${d}")

    def execute(self):
        curTime = datetime.datetime.now()
        deltaTime = (curTime - self.lastTime).total_seconds()
        curError = self.setPoint - self.input

        self.totalError += curError * deltaTime
        if self.totalError > self.windup:
            self.totalError = self.windup
        if self.totalError < -self.windup:
            self.totalError = -self.windup

        delta_error = curError - self.lastError
        dComponent = delta_error / deltaTime
        self.output = -self.p * curError - self.i * \
            self.totalError - self.d * dComponent

        pidLogger.info(
            f"p-comp: ${self.p * curError}, i-comp: ${self.i * self.totalError}, d: ${self.d * dComponent}")
