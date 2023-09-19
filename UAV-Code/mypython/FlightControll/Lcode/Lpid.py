import simple_pid
class PID:
    def __init__(self,type=0,target=0) -> None:
        self.xyp=0.7
        self.xyi=0.002
        self.xyd=0.00
        self.yawp=1.5
        self.yawi=0.0
        self.yawd=0.3
        self.xylimit=40
        self.yawlimit=30
        if type==0:
            self.pid=simple_pid.PID(self.xyp,self.xyi,self.xyd,target)
            self.pid.output_limits=(-self.xylimit,self.xylimit)
        else:
            self.pid=simple_pid.PID(self.yawp,self.yawi,self.yawd,target)
            self.pid.output_limits=(-self.yawlimit,self.yawlimit)
        pass
    def get_pid(self,current):
        return self.pid(current)
class PID2:
    def __init__(self,type=0,target=0) -> None:
        self.xyp=0.25
        self.xyi=0.003
        self.xyd=0.007
        self.yawp=1.5
        self.yawi=0.0
        self.yawd=0.3
        self.xylimit=25
        self.yawlimit=30
        if type==0:
            self.pid=simple_pid.PID(self.xyp,self.xyi,self.xyd,target)
            self.pid.output_limits=(-self.xylimit,self.xylimit)
        else:
            self.pid=simple_pid.PID(self.yawp,self.yawi,self.yawd,target)
            self.pid.output_limits=(-self.yawlimit,self.yawlimit)
        pass
    def get_pid(self,current):
        return int(self.pid(current))
class PID3:
    def __init__(self,type=0,target=0) -> None:
        self.xyp=0.25
        self.xyi=0.003
        self.xyd=0.007
        self.yawp=1.5
        self.yawi=0.0
        self.yawd=0.3
        self.xylimit=20
        self.yawlimit=15
        if type==0:
            self.pid=simple_pid.PID(self.xyp,self.xyi,self.xyd,target)
            self.pid.output_limits=(-self.xylimit,self.xylimit)
        else:
            self.pid=simple_pid.PID(self.yawp,self.yawi,self.yawd,target)
            self.pid.output_limits=(-self.yawlimit,self.yawlimit)
        pass
    def get_pid(self,current):
        return int(self.pid(current)*0.5)