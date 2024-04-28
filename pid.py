import numpy as np

class PID():
    def __init__(self, Kp, Ki, Kd, Ts):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.Ts = Ts

    def pid_update(self, error, prev_error):
        pid_prop = self.Kp * error
        pid_int = self.Ki * (error-prev_error)
        pid_der = self.Kd * (1)
        return pid_prop + pid_int + pid_der


def main():
    Ts = 0.01
    SIMTIME = 10
    simsize = 10/0.01
    pid = PID(100,0,0,0.01)
    A = np.matrix([[0, 1],[0, -2]])
    B = np.matrix([[0],[2]])
    C = 1

    x_des = 1
    x = np.array([0,0]).reshape(2,-1)
    xout = [x]
    prev_err_sig = x[0]-x_des

    for i in range(int(simsize)):
        err_sig = x[0]-x_des
        u = pid.pid_update(error=err_sig, prev_error=prev_err_sig)
        xder = np.matmul(A, x) + B*u
        x = x+xder*Ts
        xout.append(x)
        prev_err_sig = err_sig

    print(np.array(xout))



main()