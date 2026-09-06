import time
import sys
import os
import math
import array
import json
from multiprocessing import Process, Array
import wx
from pathlib import Path

   
try:
    with open('/sys/firmware/devicetree/base/model') as model:     
        RPi_model = model.read()
    if RPi_model[:12]== "Raspberry Pi":
        SIMULATION = 5
        pass                         # will be running on Raspberry Pi
    else:
        # will be running on desktop computer
        import random
        SIMULATION = 1
except Exception:
    # will be running on desktop computer
    import random                # 0 - Simulation without physics, 
    SIMULATION = 1               # 1 - Simulation synchronous with physics, 
                                        # 3 - Simulation streaming with physics

GUI_DIR = Path(__file__).resolve().parent
REPO_ROOT = GUI_DIR.parents[1]
cwd = str(REPO_ROOT)

class Soccer_monitor(wx.Frame):

    def __init__(self, *args, **kw):
        #super().__init__(*args, **kw)
        super(Soccer_monitor, self).__init__(None)
        self.pf_coord = [0,0,0]
        self.coord_odometry = [0,0,0]
        self.ball_coord = [0,0]
        self.isLeftDown = False
        self.timer = wx.Timer(self)
        self.InitUI()
        self.timer.Start(1000)

    def InitUI(self):
        self.Bind(wx.EVT_TIMER, self.OnTimer)
        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_LEFT_DOWN, self.OnLeftDown)
        self.Bind(wx.EVT_LEFT_UP, self.OnLeftUp)
        self.Bind(wx.EVT_MOTION, self.OnMove)

        self.SetTitle('Monitor')
        self.Centre()

    def OnTimer(self, event):
        if SIMULATION == 5:
            filename = '/dev/shm/monitor.json'
        else: 
            filename = cwd + "/Soccer/log/monitor.json"
        if os.path.isfile(filename):
            with open(filename, 'r') as f:
                data = json.loads(f.read())
                self.pf_coord = data['pf_coord']
                self.ball_coord = data['ball']
                self.coord_odometry = data['coord_odometry']
        self.Refresh()

    def OnPaint(self, e):
        self.dc = wx.PaintDC(self)
        #dc.SetBackground(wx.Brush('#1ac500'))
        #dc.SetPen(wx.Pen('#d4d4d4'))
        self.SetClientSize(800, 600)
        self.dc.SetBrush(wx.Brush('#1ac500'))
        self.dc.DrawRectangle(0, 0, 800, 600)
        pen = wx.Pen('#ffffff', 10, wx.SOLID)
        pen.SetJoin(wx.JOIN_MITER)
        self.dc.SetPen(pen)
        self.dc.DrawRectangle(40, 40, 720, 520)
        self.dc.DrawRectangle(0, 200, 40, 200)
        self.dc.DrawRectangle(760, 200, 40, 200)
        self.dc.DrawRectangle(40, 160, 40, 280)
        self.dc.DrawRectangle(720, 160, 40, 280)
        self.dc.DrawCircle(400,300,60)
        self.dc.DrawLine(400,45,400,555)
        self.dc.DrawLine(390,300,410,300)
        self.dc.DrawLine(210,300,230,300)
        self.dc.DrawLine(220,290,220,310)
        self.dc.DrawLine(570,300,590,300)
        self.dc.DrawLine(580,290,580,310)
        
        self.dc.SetAxisOrientation(True, True)
        self.dc.SetDeviceOrigin(400, 300)

        pen1 = wx.Pen('#000000', 1, wx.SOLID)
        self.dc.SetPen(pen1)
        self.dc.SetBrush(wx.Brush('#ffffff'))
        self.dc.DrawCircle(int(self.pf_coord[0] *200), int(self.pf_coord[1] *200) , int(0.1 *200))  #robot
        #self.dc.SetBrush(wx.Brush('#000000'))
        #for obstacle in self.obstacles:
        #    self.dc.DrawCircle(int(obstacle[0] *200), int(obstacle[1] *200), int(obstacle[2] *100))      # obstacle
        self.dc.SetBrush(wx.Brush('#ff0000'))
        self.dc.DrawCircle(int(self.ball_coord[0] *200), int(self.ball_coord[1] *200), int(0.04 *200))   # ball

        self.draw_player(self.pf_coord)
        self.draw_player(self.coord_odometry)
        #dc.Bind()

    def draw_player(self, coord):
        x, y, yaw = coord
        self.dc.SetBrush(wx.Brush('#000000'))
        width = height = 40
        start = math.degrees(yaw) + 45
        end = math.degrees(yaw) - 45
        self.dc.DrawEllipticArc(int(x * 200) - 20, int(y * 200) - 20, width, height, start, end)

    def draw_arc(self, x1, y1, x2, y2, cx, cy, CW):
        self.dc.SetBrush(wx.Brush('#ff0000', wx.BRUSHSTYLE_TRANSPARENT))
        if CW: self.dc.DrawArc(x2, y2, x1, y1, cx, cy)
        else: self.dc.DrawArc(x1, y1, x2, y2, cx, cy)

    def OnLeftDown(self, event):
        #dc = wx.ClientDC(self.staticBMP)
        pos = event.GetLogicalPosition(self.dc)
        self.isLeftDown = True
        if math.sqrt((pos[0]-self.pf_coord[0]*200)**2 + (pos[1]-self.pf_coord[1]*200)**2) <= 20: 
            self.moving_object = -1       #'robot'
            self.dx = self.pf_coord[0]*200 - pos[0]
            self.dy = self.pf_coord[1]*200 - pos[1]
        elif math.sqrt((pos[0]-self.ball_coord[0]*200)**2 + (pos[1]-self.ball_coord[1]*200)**2) <= 8: 
            self.moving_object = -2    #'ball'
            self.dx = self.ball_coord[0]*200 - pos[0]
            self.dy = self.ball_coord[1]*200 - pos[1]
        else: self.isLeftDown = False
        #if not self.isLeftDown:
        #    for obstacle in self.obstacles:
        #        if math.sqrt((pos[0]-obstacle[0]*200)**2 + (pos[1]-obstacle[1]*200)**2) <= 20:
        #            self.isLeftDown = True
        #            self.moving_object = self.obstacles.index(obstacle)      #'obstacle'
        #            self.dx = obstacle[0]*200 - pos[0]
        #            self.dy = obstacle[1]*200 - pos[1]
        #            break
        #dc.DrawCircle(pos[0], pos[1], 5)
        a = 1
        
    def OnLeftUp(self, event):
        self.isLeftDown = False

    def OnMove(self, event):
        if self.isLeftDown:
            pos = event.GetLogicalPosition(self.dc)
            if self.moving_object == -1:                         #'robot'
                self.pf_coord = [(pos[0] + self.dx)/200, (pos[1] + self.dy)/200, self.pf_coord[2]]
                self.Refresh()
            if self.moving_object == -2:                          # 'ball'
                self.ball_coord = [(pos[0] + self.dx)/200, (pos[1] + self.dy)/200]
                #self.obstacles[0] = [self.ball_coord[0], self.ball_coord[1], 0.15]
                self.Refresh()
            #if self.moving_object >= 0:                             # 'obstacle'
            #    self.obstacles[self.moving_object] = [(pos[0] + self.dx)/200, (pos[1] + self.dy)/200, self.obstacles[self.moving_object][2]]
            #    self.Refresh()

def launcher():
    app = wx.App()
    ex = Soccer_monitor(None)
    ex.Show()
    app.MainLoop()




if __name__ == '__main__':
    launcher()
