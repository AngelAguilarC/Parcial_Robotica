

#*****************************************************************#
#                      增量式PID                                  #
#*****************************************************************#
class IncrementalPID:
    def __init__(self, P, I, D, target=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
 
        self.PID_Output = 0.0            # PID控制器输出
        self.Target_Vaule = target       # 系统目标值
        self.Output_Max = 0
        self.Output_Min = 0
        self.Limit_Output = False

        self.Error = 0.0                 # 偏差
        self.LastError = 0.0
        self.LastLastError = 0.0


    # 设置PID控制器参数
    def calculate(self, nowValue):
        self.Error = nowValue - self.Target_Vaule
        IncrementValue = self.Kp * (self.Error - self.LastError) +\
        self.Ki * self.Error +\
        self.Kd * (self.Error - 2 * self.LastError + self.LastLastError)

        self.PID_Output += IncrementValue
        self.LastLastError = self.LastError
        self.LastError = self.Error
        
        if self.Limit_Output and self.PID_Output > self.Output_Max:
            self.PID_Output = self.Output_Max
        if self.Limit_Output and self.PID_Output < self.Output_Min:
            self.PID_Output = self.Output_Min
        return self.PID_Output

    def set_target(self, target):
        self.Target_Vaule = target

    def set_limit_output(self, min, max):
        if min == 0 and max == 0:
            self.Limit_Output = False
            self.Output_Min = 0
            self.Output_Max = 0
        else:
            self.Limit_Output = True
            self.Output_Min = min
            self.Output_Max = max
    
    def set_pid_param(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.PID_Output = 0
        self.Error = 0.0
        self.LastError = 0.0
        self.LastLastError = 0.0