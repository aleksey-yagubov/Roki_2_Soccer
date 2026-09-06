import wx
import wx.lib.agw.rulerctrl, wx.lib.intctrl, wx.lib.masked.ipaddrctrl, wx.adv
import sys

import io
#import serial, serial.tools.list_ports, socket, time
#import numpy as np
import random
import json
import threading
import os
#import pkg_resources.py2_warn
import time
import math
import datetime
from pathlib import Path

GUI_DIR = Path(__file__).resolve().parent
REPO_ROOT = GUI_DIR.parents[1]
current_work_directory = str(GUI_DIR).replace('\\', '/') + '/'
sys.path.insert(0, str(REPO_ROOT))
sys.path.append(str(REPO_ROOT / 'Soccer' / 'Motion'))

from Robots.class_Robot_Roki_2 import Robot
import starkit

SIMULATION = 0
try:
    with open('/sys/firmware/devicetree/base/model') as model:     
        RPi_model = model.read()
    if RPi_model[:12]== "Raspberry Pi":
        SIMULATION = 5                          # will be running on Raspberry Pi
    else:
        SIMULATION = 0                          # desktop/offline mode, no CoppeliaSim binding
except Exception:
    SIMULATION = 0                      # 0 - desktop/offline mode


class RedirectText(object):
    def __init__(self,aWxTextCtrl):
        self.out = aWxTextCtrl

    def write(self,string):
        self.out.WriteText(string)


class Pose_Designer(wx.Frame, Robot):

    def __init__(self, *args, **kw):
        super(Pose_Designer, self).__init__(*args, **kw)
        Robot.__init__(self)

        self.dof = len(self.ACTIVESERVOS)        #23
        
        self.slot_file_is_loaded = False
        self.TIK2RAD = 0.00058909
        self.syncro = False
        self.physicsOn = False
        self.panel = wx.Panel(self)
        self.panel2 = wx.Panel(self)
        self.robotPanel = wx.Panel(self)
        self.angles = []
        self.side = 0
        self.x = 0
        self.y = -53
        self.z = -233
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.xl = 0
        self.yl = 53
        self.zl = -233
        self.pitchl = 0
        self.rolll = 0
        self.yawl = 0
        self.joint5 = 0
        self.joint6 = 0
        self.joint7 = 0
        self.joint8 = 0
        self.joint9 = 0
        self.joint10 = 0
        if SIMULATION == 5:
            self.F = 1.5
        else:
            self.F = 1
        
        self.focused_item = 0
        self.dummy_handles_list_incomplete = True

        self.filename = ''
        with open(current_work_directory + "pose_designer_config.json", "r") as f:
                self.config = json.loads(f.read())
        #self.config = {'defaultFile':''}
        self.defaultFile = self.config['defaultFile']
        self.jointControls = []
        #self.controlBoxPosition = [(10,0), (9,1), (8,1), (7,1),(6,0), (5,1),(4,0),(3,0),(2,0), (2,1), (4,2), (10,4), (9,3),(8,3), (7,3), (6,4), (5,3), (4,4), (3,4), (2,4), (2,3), (1,2),(0,2)]
        self.controlBoxPosition1 = [(55, 500), (120, 450), (80, 400), (140, 350), (140, 300), (140, 250), (5, 245), (5, 195), (5, 145), (140, 145), (210, 200),
                                   (365, 500), (300, 450), (340, 400), (280, 350), (280, 300), (280, 250), (415, 245), (415, 195), (415, 145), (280, 145),
                                   (210, 95), (210,45), (5, 295), (415, 295), (5, 345), (415, 345)]
        # self.ACTIVEJOINTS =        ['Leg_right_10','Leg_right_9','Leg_right_8','Leg_right_7','Leg_right_6','Leg_right_5','hand_right_4',
        #                            'hand_right_3','hand_right_2','hand_right_1','Tors1',
        #                            'Leg_left_10','Leg_left_9','Leg_left_8',
        #                            'Leg_left_7','Leg_left_6','Leg_left_5','hand_left_4','hand_left_3','hand_left_2','hand_left_1','head0','head12', 'hand_right_11', 'hand_left_11']
        
        self.jointHandle = []
        self.pose0 = [0 for i in range(27)]
        self.trims = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.activePose = [0 for i in range(27)]
        self.activePoseOld = self.activePose.copy()
        self.activeFrames = 1
        self.motionPages = [[]]
        self.pageNames = []
        self.activePage = 0
        self.new_file_is_Not_saved = False
        self.slow = 0
        self.dummy_handle_list = []
        self.dummy_handles = []
        
        if SIMULATION == 5 :
            class Glob:
                def __init__(self):
                    self.Roki = None
            glob = Glob()
            import Roki
            self.Roki = Roki
            from Soccer.Motion.class_stm_channel import STM_channel
            self.stm_channel = STM_channel(glob)
            self.rcb = self.stm_channel.rcb
        else:
            self.clientID = sim_Enable('127.0.0.1', -20000)
        self.sim_Start()
        self.InitUI()

    def sim_Start(self):
        if SIMULATION == 1 or SIMULATION  == 0:
            dataType = 0
            objectType = sim.sim_object_joint_type
            returnCode, joint_handles, intdata, floatdata, stringdata = sim.simxGetObjectGroupData(self.clientID, objectType, dataType, sim.simx_opmode_blocking)
            for jointname in stringdata:
                if jointname == 'hand_right_11': self.dof = 25
            print(self.dof)
            for i in range(self.dof):
                returnCode, handle= sim.simxGetObjectHandle(self.clientID, self.ACTIVEJOINTS[i], sim.simx_opmode_blocking)
                self.jointHandle.append(handle)
                returnCode, position= sim.simxGetJointPosition(self.clientID, handle, sim.simx_opmode_blocking)
                self.activePose[i] = position / self.TIK2RAD

            if SIMULATION == 1:
                sim.simxSynchronous(self.clientID,True)
            

    def action_To_Robot(self):
        if self.syncro:
            if SIMULATION == 1 or SIMULATION  == 0:
                if self.physicsOn:
                    for i in range(self.dof):
                        returnCode = sim.simxSetJointTargetPosition(self.clientID, self.jointHandle[i] ,
                                             (self.activePose[i] + self.trims[i]) * self.TIK2RAD * self.ACTIVESERVOS[i][3], sim.simx_opmode_oneshot)
                    sim.simxSynchronousTrigger(self.clientID)
                else:
                    for i in range(self.dof):
                        returnCode = sim.simxSetJointPosition(self.clientID, self.jointHandle[i] ,
                                             (self.activePose[i] + self.trims[i]) * self.TIK2RAD * self.ACTIVESERVOS[i][3], sim.simx_opmode_oneshot)
            if SIMULATION == 5:
                servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(self.dof + 2)]
                for i in range(self.dof):
                    if self.ACTIVESERVOS[i][0] == 8:
                        n = self.dof - 1 + self.ACTIVESERVOS[i][1]
                        pos = int((self.activePose[i] + self.trims[i]) * self.ACTIVESERVOS[i][2]/2 + 7500)
                        servoDatas[n].Id, servoDatas[n].Sio, servoDatas[n].Data = 13, self.ACTIVESERVOS[i][1], pos
                    else: pos = int((self.activePose[i] + self.trims[i]) * self.ACTIVESERVOS[i][2] + 7500)
                    servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                a=self.rcb.setServoPosAsync(servoDatas, 2, 1)
        
    def createControlPanel(self, jointNumber):
        position = (int(self.controlBoxPosition1[jointNumber][0]*self.F), int(self.controlBoxPosition1[jointNumber][1]*self.F))
        pnl = wx.Panel(self.robotPanel, pos=position, size = (int(130*self.F),int(50*self.F)), style = wx.TAB_TRAVERSAL|wx.BORDER_RAISED)
        #pnl.SetBackgroundColour(wx.SystemSettings.GetColour(wx.SYS_COLOUR_MENU))
        pnl.SetBackgroundColour(wx.SystemSettings.GetColour(wx.SYS_COLOUR_MENU))
        val1 = 0
        controlName = wx.StaticText(pnl, label=self.ACTIVEJOINTS[jointNumber], pos = (20*self.F,0), size=(int(60*self.F),int(15*self.F)), style= wx.ALIGN_LEFT)
        controlslider = wx.Slider(pnl, id = jointNumber,   value=val1, minValue=-8000, maxValue=8000, pos = (0,int(30*self.F)), size = (int(130*self.F),int(20*self.F)))
        controlvalue = wx.SpinCtrl(pnl, id = jointNumber, value="0", pos=(int(15*self.F),15*self.F), size= (int(100*self.F), int(15*self.F)),  #wx.DefaultSize,
                                  #style=wx.SP_WRAP, min=self.jointLimits[jointNumber][0], max=self.jointLimits[jointNumber][1], initial=0)
                                  style=wx.SP_ARROW_KEYS|wx.ALIGN_LEFT, min=self.ACTIVESERVOS[jointNumber][4], max=self.ACTIVESERVOS[jointNumber][5], initial=0)
                                  #style=wx.ALIGN_LEFT, min=self.ACTIVESERVOS[jointNumber][4], max=self.ACTIVESERVOS[jointNumber][5], initial=0)
        #controlvalue.SetIncrement(10)
        controlslider.Bind(wx.EVT_SLIDER, self.On_Slider_move)
        controlvalue.Bind(wx.EVT_SPINCTRL, self.On_SPINCTRL_change)
        controlvalue.Bind(wx.EVT_TEXT, self.On_SPINCTRL_change)
        return pnl, controlslider, controlvalue
        
    def refresh_Control_Values(self):
        if len(self.dummy_handles) > 0:
            self.Dummy_Fetch_Data(0)
        for i in range(self.dof):
            self.jointControls[i][1].SetValue(self.activePose[i])
            self.jointControls[i][2].SetValue(self.activePose[i])
        self.frames_input.SetValue(self.activeFrames)

    def InitUI(self):
       
        for i in range(self.dof):
        #for i in range(17):
            pnl, controlslider, controlvalue = self.createControlPanel(i)
            self.jointControls.append((pnl, controlslider, controlvalue))

        pnl2 = wx.Panel(self.robotPanel, pos=(int(10* self.F),int(10* self.F)), size = (int(80* self.F),int(50* self.F)), style = wx.TAB_TRAVERSAL)
        self.frames_text = wx.StaticText(pnl2, label='Frames', pos = (0,0), size=(int(60* self.F),int(25* self.F)), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.frames_input = wx.lib.intctrl.IntCtrl(pnl2, value=self.activeFrames, pos =(0,int(25* self.F)), size=(int(60* self.F),int(25* self.F)), min=1, max=200)
        self.robotPanel.SetBackgroundColour(wx.Colour('WHEAT'))

        self.pages_edit_list_control = wx.adv.EditableListBox(self.robotPanel, label = "Pages", pos = (int(345 * self.F), 0), size = (int(200*self.F), int(140*self.F)),
                                                             style=wx.adv.EL_DEFAULT_STYLE)
        self.getNew_button = self.pages_edit_list_control.GetNewButton()
        self.get_Del_button = self.pages_edit_list_control.GetDelButton()
        self.get_Up_button = self.pages_edit_list_control.GetUpButton()
        self.get_Down_button = self.pages_edit_list_control.GetDownButton()
        self.get_Up_button.Bind(wx.EVT_BUTTON, self.On_MoveUp_Page)
        self.get_Down_button.Bind(wx.EVT_BUTTON, self.On_MoveDn_Page)
        self.get_Del_button.Bind(wx.EVT_BUTTON, self.On_Delete_Page)
        self.getNew_button.Bind(wx.EVT_BUTTON, self.On_Clone_Page)
        self.pages_edit_list_control.Bind(wx.EVT_LIST_ITEM_FOCUSED, self.On_Left_Down_edit)
        self.pages_edit_list_control.Bind(wx.EVT_LIST_END_LABEL_EDIT, self.On_Label_edit)

        self.console_Panel = wx.Panel(self)
        self.log = wx.TextCtrl(self.console_Panel, -1, style=wx.TE_MULTILINE)
        log_box = wx.BoxSizer(wx.VERTICAL)
        log_box.Add(self.log, proportion = 1, flag=wx.EXPAND|wx.BOTTOM|wx.TOP)
        self.console_Panel.SetSizer(log_box)
        redir = RedirectText(self.log)
        #sys.stdout = redir
        #sys.stderr = redir
        self.CreateMenuBar()

        self.denied_message = wx.MessageDialog(self, "Denied! No motion slot processing", style=wx.OK|wx.CENTRE)
        self.physics_button = wx.CheckBox(self.panel2, wx.ID_ANY , label = "PhysicsON", style= wx.RB_SINGLE | wx.ALIGN_RIGHT)
        self.physics_button.SetValue(self.physicsOn)
        self.play_page_button = wx.Button(self.panel, wx.ID_ANY, "PlayPage")
        self.play_next_button = wx.Button(self.panel, wx.ID_ANY, "PlayNext")
        self.record_page_button = wx.Button(self.panel, wx.ID_ANY, "RecordPage")
        self.play_all_button = wx.Button(self.panel, wx.ID_ANY, "PlayAll")
        self.return_button = wx.Button(self.panel, wx.ID_ANY, "Return")
        self.syncro_button = wx.Button(self.panel, id = 1001, label = "NOSYNC")
        self.pose0_button = wx.Button(self.panel2, id = 1002, label = "Pose0")
        self.discard_button = wx.Button(self.panel2, id = 1003, label = "Discard")
        self.to_page0_button = wx.Button(self.panel2, id = 1004, label = "To page0")
        self.slow_text = wx.StaticText(self.panel2, label='Slow(ms):',size=(int(60* self.F),int(25* self.F)), style= wx.ALIGN_CENTRE_HORIZONTAL)
        self.slow_input = wx.lib.intctrl.IntCtrl(self.panel2, value=self.slow, size=(int(60* self.F),int(25* self.F)), min=0, max=5000)

        self.button_box = wx.BoxSizer(wx.HORIZONTAL)
        self.button_box2 = wx.BoxSizer(wx.HORIZONTAL)

        self.button_box2.Add(self.physics_button, proportion=1)
        self.button_box.Add(self.return_button, proportion=1)
        self.button_box.Add(self.play_page_button, proportion=1)
        self.button_box.Add(self.play_next_button, proportion=1)
        self.button_box.Add(self.play_all_button, proportion=1)
        self.button_box.Add(self.syncro_button, proportion=1)
        self.button_box.Add(self.record_page_button, proportion=1)

        self.button_box2.Add(self.slow_text, proportion=1, flag=wx.RIGHT )
        self.button_box2.Add(self.slow_input, proportion=1, flag=wx.RIGHT )

        self.button_box2.Add(self.pose0_button, proportion=1)
        self.button_box2.Add(self.discard_button, proportion=1)
        self.button_box2.Add(self.to_page0_button, proportion=1)

        self.panel.SetSizer(self.button_box)
        self.panel2.SetSizer(self.button_box2)
        
        self.button_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.button_sizer2 = wx.BoxSizer(wx.HORIZONTAL)
        self.button_sizer.Add(self.panel, proportion=2, flag=wx.EXPAND  )
        self.button_sizer2.Add(self.panel2, proportion=2, flag=wx.EXPAND  )
        

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.button_sizer, proportion=0, flag=wx.EXPAND )
        sizer.Add(self.button_sizer2, proportion=0, flag=wx.EXPAND )
        sizer.Add(self.robotPanel, proportion=0, flag=wx.EXPAND )

        #self.whole_window = wx.BoxSizer(wx.HORIZONTAL)
        #self.whole_window.Add(sizer, proportion = 0, flag=wx.EXPAND )
        #self.whole_window.Add(self.console_Panel, proportion = 1, flag=wx.EXPAND|wx.BOTTOM|wx.TOP  )

        self.SetMinSize((350, 350))
        self.CreateStatusBar()
        #self.SetSizer(self.whole_window)
        self.SetSizer(sizer)

        #self.SetSize((560, 700))
        self.SetSize((int(560 * self.F), int(700 * self.F)))
        self.SetTitle('Pose Designer')
        self.Centre()

        self.Bind(wx.EVT_SIZE, self.On_Change_Window_Size)
        self.Bind(wx.EVT_MENU, self.On_New_Slot, id = 100 )
        self.Bind(wx.EVT_MENU, self.On_Load_File, id = 101 )
        self.Bind(wx.EVT_MENU, self.On_Save, id = 102 )
        self.Bind(wx.EVT_MENU, self.On_Save_as, id = 103 )
        self.Bind(wx.EVT_MENU, self.On_Quit_select, id = 105 )
        self.Bind(wx.EVT_MENU, self.OnAbout, id = 121 )
        self.Bind(wx.EVT_MENU, self.Quick_Start, id = 122 )
        self.Bind(wx.EVT_MENU, self.Calculator, id = 123 )
        self.Bind(wx.EVT_MENU, self.Mirror_Transform, id = 124 )
        self.Bind(wx.EVT_MENU, self.ListBox_test, id = 125 )
        self.Bind(wx.EVT_MENU, self.Dummy_report, id = 127 )
        self.Bind(wx.EVT_MENU, self.Rename_All_Pages, id = 126 )
        #self.Bind(wx.EVT_MENU, self.On_Start_Camera, id = 3 )
        self.physics_button.Bind(wx.EVT_CHECKBOX, self.On_Physics)
        #self.quit_button.Bind(wx.EVT_BUTTON, self.On_Quit_select)
        self.return_button.Bind(wx.EVT_BUTTON, self.On_Return )
        self.play_page_button.Bind(wx.EVT_BUTTON, self.On_Play_Page )
        self.play_next_button.Bind(wx.EVT_BUTTON, self.On_Play_Next )
        self.record_page_button.Bind(wx.EVT_BUTTON, self.On_Record_Page )
        self.play_all_button.Bind(wx.EVT_BUTTON, self.On_Play_All )
        self.syncro_button.Bind(wx.EVT_BUTTON, self.On_Syncro_Set )
        self.pose0_button.Bind(wx.EVT_BUTTON, self.On_Pose0_Set )
        self.discard_button.Bind(wx.EVT_BUTTON, self.On_Discard_Set )
        self.to_page0_button.Bind(wx.EVT_BUTTON, self.On_To_page0_Set )
        self.frames_input.Bind(wx.EVT_TEXT, self.On_number_input)
        self.slow_input.Bind(wx.EVT_TEXT, self.On_number_input)

    def On_Change_Window_Size(self, event):
        new_window_size = event.GetSize()
        print('New Size : ', new_window_size)
        #self.F = min(new_window_size[0]/560, new_window_size[1]/700)
        #self.SetSize((int(560 * self.F), int(700 * self.F)))


    def Rename_All_Pages(self, event):
        if self.slot_file_is_loaded == False:
            self.denied_message.ShowModal()
            return
        new_names = []
        for i in range(len(self.pageNames)):
            name = 'page ' + str(i)
            new_names.append(name)
        self.pageNames = new_names
        self.pages_edit_list_control.SetStrings(self.pageNames)
        pages_edit_list_control = self.pages_edit_list_control.GetListCtrl()
        pages_edit_list_control.Select(self.activePage)
        pages_edit_list_control.EnsureVisible(self.activePage)
        print(" All pages renamed")

    def Dummy_report_Appoint_Focused_Item(self, event):
            self.focused_item = self.dummy_report_table.GetFocusedItem()
            print("focused item chosen")

    def Dummy_Fetch_Data(self, event):
        if self.dummy_handles_list_incomplete :
            for i in range(len(self.dummy_handles)):
                if self.dummy_report_table.IsItemChecked(i): self.dummy_handles_checked[i] = True
                else: self.dummy_handles_checked[i] = False
            self.dummy_report_table.EnableCheckBoxes(False)
            self.dummy_handles_list_incomplete = False
        dataType = 3
        objectType = sim.sim_object_dummy_type
        returnCode, self.dummy_handles, intdata, floatdata, stringdata = sim.simxGetObjectGroupData(self.clientID, objectType, dataType, sim.simx_opmode_blocking)
        print("Sim Group Data:", "intdata:", intdata, "floatdata:", floatdata, "stringdata", stringdata)
        self.dummy_report_table.DeleteAllItems()
        for i in range(len(self.dummy_handle_list)):
            if self.dummy_handles_checked[i]:
                content = []
                content.append(self.dummy_handle_list[i])
                content.append(str(int(floatdata[i*3] * 1000)))
                content.append(str(int(floatdata[i*3 +1]* 1000)))
                content.append(str(int(floatdata[i*3 +2]*1000)))
                self.dummy_report_table.Append(content)
        self.dummy_report_table.Select(self.focused_item)
        self.dummy_report_table.EnsureVisible(self.focused_item)
        print("Update button pressed")

    def Dummy_report(self, event):
        dialog_panel = wx.Dialog(self, title='Dummy Report', size = (340, 165))
        self.dummy_report_table = wx.ListCtrl(dialog_panel, size = (320, 100), style = wx.LC_REPORT | wx.LC_VRULES | wx.LC_HRULES)
        refresh_button = wx.Button(dialog_panel, label = "Update",  pos =(10, 100))
        refresh_button.Bind(wx.EVT_BUTTON, self.Dummy_Fetch_Data)
        self.dummy_report_table.InsertColumn(0,'Dummy #')
        self.dummy_report_table.InsertColumn(1,'X')
        self.dummy_report_table.InsertColumn(2,'Y')
        self.dummy_report_table.InsertColumn(3,'Z')
        self.dummy_report_table.SetColumnWidth(0, 150)
        self.dummy_report_table.SetColumnWidth(1, 50)
        self.dummy_report_table.SetColumnWidth(2, 50)
        self.dummy_report_table.SetColumnWidth(3, 50)
        self.dummy_report_table.EnableCheckBoxes(True)
        objectType = sim.sim_object_dummy_type
        dataType = 0
        returnCode, self.dummy_handles, intdata, floatdata, self.dummy_handle_list = sim.simxGetObjectGroupData(self.clientID, objectType, dataType, sim.simx_opmode_blocking)
        dataType = 3
        returnCode, self.dummy_handles, intdata, floatdata, stringdata = sim.simxGetObjectGroupData(self.clientID, objectType, dataType, sim.simx_opmode_blocking)
        print("Sim Group Data:", "intdata:", intdata, "floatdata:", floatdata, "stringdata", stringdata)
        self.dummy_handles_checked = []
        self.dummy_handles_list_incomplete = True
        for i in range(len(self.dummy_handle_list)):
            content = []
            content.append(self.dummy_handle_list[i])
            content.append(str(int(floatdata[i*3] * 1000)))
            content.append(str(int(floatdata[i*3 +1]* 1000)))
            content.append(str(int(floatdata[i*3 +2]*1000)))
            self.dummy_report_table.Append(content)
            self.dummy_report_table.CheckItem(i, False)
            self.dummy_handles_checked.append(True)
        self.dummy_report_table.Bind(wx.EVT_LIST_ITEM_FOCUSED, self.Dummy_report_Appoint_Focused_Item)
        dialog_panel.Show()

    def ListBox_test(self, event):
        def On_List_edit(event):
            #item, flags = pages_list_control.HitTest(event.GetPosition())
            #print(item)
            pages_list_control = pages_edit_list_control.GetListCtrl()
            print(pages_list_control.GetFirstSelected())
        def On_List_inserted(event):
            print('new list item')
        def On_List_change(event):
            pages_list_control = pages_edit_list_control.GetListCtrl()
            print(pages_list_control.GetItem())
        def On_GetUp(event):
            print('getup button pressed')
        dialog_panel = wx.Dialog(self, title='ListBox Test', size = (150, 250))
        pages_edit_list_control = wx.adv.EditableListBox(dialog_panel, pos = (10, 40), size = (150, 250), style=wx.adv.EL_DEFAULT_STYLE)
        strings =['one', 'two', "three"]
        pages_edit_list_control.SetStrings(strings)
        #pages_edit_list_control.Bind(wx.EVT_LEFT_DOWN, On_List_edit)
        #pages_list_control = pages_edit_list_control.GetListCtrl()
        getup_button = pages_edit_list_control.GetUpButton()
        getup_button.Bind(wx.EVT_BUTTON, On_GetUp)
        pages_edit_list_control.Bind(wx.EVT_LIST_ITEM_FOCUSED, On_List_edit)
        pages_edit_list_control.Bind(wx.EVT_LIST_INSERT_ITEM, On_List_inserted)
        pages_edit_list_control.Bind(wx.EVT_LIST_BEGIN_DRAG, On_List_change)
        dialog_panel.ShowModal()

    def Mirror_Transform(self, event):
        def On_Select_All(event):
            if select_all_checkbox.IsChecked():
                for i in range(len(self.motionPages)):
                    pages_list_control.CheckItem(i, True)
            else:
                for i in range(len(self.motionPages)):
                    pages_list_control.CheckItem(i, False)
        def On_Transform(event):
            transform_list = []
            for i in range(len(self.motionPages)):
                if pages_list_control.IsItemChecked(i): transform_list.append(i)
            for item in transform_list:
                itemtext = str(item) + ' Mirrored'
                pages_list_control.SetItemText(item, itemtext)
            for item in transform_list:
                side_copy =  self.motionPages[item][1:11].copy()
                for i in range(10):
                    self.motionPages[item][i+1] = -self.motionPages[item][i+12]
                    self.motionPages[item][i+12] = -side_copy[i]
                self.motionPages[item][11] = -self.motionPages[item][11]
                self.motionPages[item][22] = -self.motionPages[item][22]

        mirror_transform_dialog = wx.Dialog(self, title='Mirror Transform', size = (350, 350))
        text2 = wx.StaticText(mirror_transform_dialog, label='Select pages for transformation',size=(300,25), pos =(10, 10), style = wx.ALIGN_CENTRE_HORIZONTAL  )
        pages_list_control = wx.ListCtrl(mirror_transform_dialog, pos = (10, 40), size = (200, 200), style=wx.LC_LIST | wx.SIMPLE_BORDER)
        select_all_checkbox = wx.CheckBox(mirror_transform_dialog, pos = (220, 40), label = "Select All")
        transform_button = wx.Button(mirror_transform_dialog, wx.ID_ANY, "Transform", pos = (10, 250))
        pages_list_control.EnableCheckBoxes(True)
        pages_list_control.DeleteAllItems()
        for i in range(len(self.motionPages)):
            pages_list_control.InsertItem(i, self.pageNames[i])
        transform_button.Bind(wx.EVT_BUTTON, On_Transform)
        select_all_checkbox.Bind(wx.EVT_CHECKBOX, On_Select_All)
        mirror_transform_dialog.ShowModal()
        



    def Calculator(self, event):
        def On_Calc_Button(event):
            self.side = side_input.GetSelection()
            self.x = x_input.GetValue()
            self.y = y_input.GetValue()
            self.z = z_input.GetValue()
            self.pitch = pitch_input.GetValue()
            self.roll = roll_input.GetValue()
            self.yaw = yaw_input.GetValue()
            self.xl = xl_input.GetValue()
            self.yl = yl_input.GetValue()
            self.zl = zl_input.GetValue()
            self.pitchl = pitchl_input.GetValue()
            self.rolll = rolll_input.GetValue()
            self.yawl = yawl_input.GetValue()
            xr = math.sin(self.pitch)
            yr = math.sin(self.roll)
            zr = - math.sqrt(1 - xr**2 - yr**2)
            wr = math.radians(self.yaw)
            xl = math.sin(self.pitchl)
            yl = math.sin(self.rolll)
            zl = - math.sqrt(1 - xl**2 - yl**2)
            wl = math.radians(self.yawl)
            if self.side == 0:
                self.angles = starkit.alpha_calculation(self.x, self.y, self.z, xr, yr, zr, wr, self.SIZES, self.LIMALPHA)
            else:
                self.angles = starkit.alpha_calculation(self.xl, -self.yl, self.zl, xl, -yl, zl, wl, self.SIZES, self.LIMALPHA)
            if len(self.angles) == 0:
                result_list_control.DeleteAllItems()
                result_list_control.InsertItem(0, "Result doesn't exist")
            else:
                result_list_control.DeleteAllItems()
                for i in range(len(self.angles)):
                    variant_text = 'Variant '+ str(i)
                    if self.angles[i][2] < 0  and self.side == 0: variant_text += "-best"
                    if self.angles[i][2] < 0  and self.side == 1: variant_text += "-best"
                    result_list_control.InsertItem(i, variant_text)
            if self.side == 0: 
                sideword = "Right"
                print( "Calc data : ", '\n'," Side = \t", sideword, '\n', " X = \t", self.x,'\n',
                    " Y = \t" , self.y, '\n', " Z = \t", self.z, '\n', " Pitch = \t", self.pitch, '\n',
                    " Roll = \t", self.roll, '\n', " Yaw = \t", self.yaw)
            else: 
                sideword = "Left"
                print( "Calc data : ", '\n'," Side = \t", sideword, '\n', " X = \t", self.xl,'\n',
                    " Y = \t" , self.yl, '\n', " Z = \t", self.zl, '\n', " Pitch = \t", self.pitchl, '\n',
                    " Roll = \t", self.rolll, '\n', " Yaw = \t", self.yawl)
        def On_Copy_Button(event):
            print('Copy button pressed')
            if side_input.GetSelection() == 0:
                self.activePose[0] = joint10_result.GetValue()
                self.activePose[1] = joint9_result.GetValue()
                self.activePose[2] = joint8_result.GetValue()
                self.activePose[3] = joint7_result.GetValue()
                self.activePose[4] = joint6_result.GetValue()
                self.activePose[5] = joint5_result.GetValue()
                print(" Copy result: ")
                for i in range(6):
                    print( self.ACTIVEJOINTS[i], " = \t", self.activePose[i])
            else:
                self.activePose[11] = -joint10_result.GetValue()
                self.activePose[12] = -joint9_result.GetValue()
                self.activePose[13] = -joint8_result.GetValue()
                self.activePose[14] = -joint7_result.GetValue()
                self.activePose[15] = -joint6_result.GetValue()
                self.activePose[16] = -joint5_result.GetValue()
                print(" Copy result: ")
                for i in range(11,17):
                    print( self.ACTIVEJOINTS[i], " = \t", self.activePose[i])
            self.refresh_Control_Values()

        def On_Result_Selection(event):
            item = event.GetIndex()
            angle10, angle9, angle8, angle7, angle6, angle5 = self.angles[item]
            self.joint5 = int(angle5/self.TIK2RAD)
            self.joint6 = int(angle6/self.TIK2RAD)
            self.joint7 = int(angle7/self.TIK2RAD)
            self.joint8 = int(angle8/self.TIK2RAD)
            self.joint9 = int(angle9/self.TIK2RAD)
            self.joint10 = int(angle10/self.TIK2RAD)
            joint5_result.SetValue(self.joint5)
            joint6_result.SetValue(self.joint6)
            joint7_result.SetValue(self.joint7)
            joint8_result.SetValue(self.joint8)
            joint9_result.SetValue(self.joint9)
            joint10_result.SetValue(self.joint10)

        calc_input = wx.Dialog(self, title='IK Calculator', size = (400, 400))
        text1 = wx.StaticText(calc_input, label='Input calc',size=(200,25))
        text2 = wx.StaticText(calc_input, label='X, mm',size=(100,25), pos =(10, 70), style = wx.ALIGN_CENTRE_HORIZONTAL  )
        text3 = wx.StaticText(calc_input, label='Y, mm',size=(100,25), pos =(10, 100), style = wx.ALIGN_CENTRE_HORIZONTAL  )
        text4 = wx.StaticText(calc_input, label='Z, mm',size=(100,25), pos =(10, 130), style = wx.ALIGN_CENTRE_HORIZONTAL  )
        text5 = wx.StaticText(calc_input, label='Pitch, degrees',size=(100,25), pos =(10, 160), style = wx.ALIGN_CENTRE_HORIZONTAL  )
        text6 = wx.StaticText(calc_input, label='Roll, degrees',size=(100,25), pos =(10, 190) , style = wx.ALIGN_CENTRE_HORIZONTAL)
        text7 = wx.StaticText(calc_input, label='Yaw, degrees',size=(100,25), pos =(10, 220), style = wx.ALIGN_CENTRE_HORIZONTAL )
        text8 = wx.StaticText(calc_input, label='Joint 5',size=(60,25), pos =(230, 70), style = wx.ALIGN_CENTRE_HORIZONTAL )
        text9 = wx.StaticText(calc_input, label='Joint 6',size=(60,25), pos =(230, 100), style = wx.ALIGN_CENTRE_HORIZONTAL )
        text10 = wx.StaticText(calc_input, label='Joint 7',size=(60,25), pos =(230, 130), style = wx.ALIGN_CENTRE_HORIZONTAL )
        text11 = wx.StaticText(calc_input, label='Joint 8',size=(60,25), pos =(230, 160), style = wx.ALIGN_CENTRE_HORIZONTAL )
        text12 = wx.StaticText(calc_input, label='Joint 9',size=(60,25), pos =(230, 190), style = wx.ALIGN_CENTRE_HORIZONTAL )
        text13 = wx.StaticText(calc_input, label='Joint 10',size=(60,25), pos =(230, 220), style = wx.ALIGN_CENTRE_HORIZONTAL )
        side_input = wx.RadioBox(calc_input, pos = (120, 20), choices = ["Right", "Left"], majorDimension = 2)
        side_input.SetSelection(self.side)
        x_input = wx.lib.intctrl.IntCtrl(calc_input, value = self.x, pos =(120, 70), size=(40,25), min=-300, max=300)
        y_input = wx.lib.intctrl.IntCtrl(calc_input, value = self.y, pos =(120, 100), size=(40,25), min=-300, max=300)
        z_input = wx.lib.intctrl.IntCtrl(calc_input, value = self.z, pos =(120, 130), size=(40,25), min=-300, max=300)
        pitch_input = wx.lib.intctrl.IntCtrl(calc_input, value = self.pitch, pos =(120, 160), size=(40,25), min=-300, max=300)
        roll_input = wx.lib.intctrl.IntCtrl(calc_input, value = self.roll, pos =(120, 190), size=(40,25), min=-300, max=300)
        yaw_input = wx.lib.intctrl.IntCtrl(calc_input,  value = self.yaw, pos =(120, 220), size=(40,25), min=-300, max=300)
        xl_input = wx.lib.intctrl.IntCtrl(calc_input, value = self.xl, pos =(170, 70), size=(40,25), min=-300, max=300)
        yl_input = wx.lib.intctrl.IntCtrl(calc_input, value = self.yl, pos =(170, 100), size=(40,25), min=-300, max=300)
        zl_input = wx.lib.intctrl.IntCtrl(calc_input, value = self.zl, pos =(170, 130), size=(40,25), min=-300, max=300)
        pitchl_input = wx.lib.intctrl.IntCtrl(calc_input, value = self.pitchl, pos =(170, 160), size=(40,25), min=-300, max=300)
        rolll_input = wx.lib.intctrl.IntCtrl(calc_input, value = self.rolll, pos =(170, 190), size=(40,25), min=-300, max=300)
        yawl_input = wx.lib.intctrl.IntCtrl(calc_input,  value = self.yawl, pos =(170, 220), size=(40,25), min=-300, max=300)
        joint5_result = wx.lib.intctrl.IntCtrl(calc_input, value = self.joint5, pos =(290, 70), size=(60,25), min=-8000, max=8000)
        joint6_result = wx.lib.intctrl.IntCtrl(calc_input, value = self.joint6, pos =(290, 100), size=(60,25), min=-8000, max=8000)
        joint7_result = wx.lib.intctrl.IntCtrl(calc_input, value = self.joint7, pos =(290, 130), size=(60,25), min=-8000, max=8000)
        joint8_result = wx.lib.intctrl.IntCtrl(calc_input, value = self.joint8, pos =(290, 160), size=(60,25), min=-8000, max=8000)
        joint9_result = wx.lib.intctrl.IntCtrl(calc_input, value = self.joint9, pos =(290, 190), size=(60,25), min=-8000, max=8000)
        joint10_result = wx.lib.intctrl.IntCtrl(calc_input, value = self.joint10, pos =(290, 220), size=(60,25), min=-8000, max=8000)
        calc_button = wx.Button(calc_input, id = 2000, label = "CALC", pos = (30,250))
        copy_button = wx.Button(calc_input, id = 2001, label = "COPY", pos = (240,250))
        result_list_control = wx.ListCtrl(calc_input, pos = (30, 300), size = (260, 35), style=wx.LC_LIST | wx.LC_SINGLE_SEL | wx.SIMPLE_BORDER)
        if len(self.angles) == 0:
                result_list_control.DeleteAllItems()
                result_list_control.InsertItem(0, "Result doesn't exist")
        else:
            result_list_control.DeleteAllItems()
            for i in range(len(self.angles)):
                variant_text = 'Variant '+ str(i)
                result_list_control.InsertItem(i, variant_text)
        calc_button.Bind(wx.EVT_BUTTON, On_Calc_Button)
        copy_button.Bind(wx.EVT_BUTTON, On_Copy_Button)
        #result_list_control.Bind(wx.EVT_LEFT_DOWN, On_Result_Selection)
        result_list_control.Bind(wx.EVT_LIST_ITEM_SELECTED, On_Result_Selection)
        calc_input.Show()

    def On_Discard_Set(self, event):
        print('Discard button pressed')
        self.activePoseOld = self.activePose.copy()

    def On_To_page0_Set(self, event):
        self.activePage = 0
        print('self.activePage:', self.activePage)
        if self.motionPages != [[]]:
            self.activeFrames = self.motionPages[0][0]
            self.activePoseOld = self.activePose.copy()
            for i in range(1, len(self.motionPages[self.activePage])):
                self.activePose[i-1] = self.motionPages[self.activePage][i]
        self.refresh_Control_Values()
        pages_list_control = self.pages_edit_list_control.GetListCtrl()
        pages_list_control.Select(self.activePage)
        pages_list_control.EnsureVisible(self.activePage)

    def On_Pose0_Set(self, event):
        print('Pose0 button pressed')
        #self.activePose = self.trims.copy()
        self.activePose = self.pose0.copy()
        self.activePoseOld = self.activePose.copy()
        self.activeFrames = 10
        self.refresh_Control_Values()
        if self.syncro:
            if SIMULATION == 1 or SIMULATION  == 0:
                if self.physicsOn:
                    for j in range(self.dof):
                        returnCode = sim.simxSetJointTargetPosition(self.clientID, self.jointHandle[j] ,
                                    (self.pose0[j] + self.trims[j]) * self.TIK2RAD * self.ACTIVESERVOS[j][3], sim.simx_opmode_oneshot)
                    sim.simxSynchronousTrigger(self.clientID)
                else:
                    for j in range(self.dof):
                        returnCode = sim.simxSetJointPosition(self.clientID, self.jointHandle[j] ,
                                    (self.pose0[j] + self.trims[j]) * self.TIK2RAD * self.ACTIVESERVOS[j][3], sim.simx_opmode_oneshot)
            if SIMULATION == 5:
                servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(self.dof + 2)]
                for i in range(self.dof):
                    if self.ACTIVESERVOS[i][0] == 8:
                        n = self.dof - 1 + self.ACTIVESERVOS[i][1]
                        pos = int((self.pose0[i] + self.trims[i]) * self.ACTIVESERVOS[i][2]/2 + 7500)
                        servoDatas[n].Id, servoDatas[n].Sio, servoDatas[n].Data = 13, self.ACTIVESERVOS[i][1], pos
                    else: pos = int((self.pose0[i] + self.trims[i]) * self.ACTIVESERVOS[i][2] + 7500)
                    servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                a=self.rcb.setServoPosAsync(servoDatas, 50, 49)



    def On_MoveUp_Page(self, event):
        if self.slot_file_is_loaded == False:
            self.denied_message.ShowModal()
            return
        if self.activePage > 0:
            page = self.motionPages.pop(self.activePage)
            name = self.pageNames.pop(self.activePage)
            self.activePage -= 1
            self.motionPages.insert(self.activePage, page)
            self.pageNames.insert(self.activePage, name)
            #self.pages_list_control.DeleteAllItems()
            #for i in range(len(self.motionPages)):
            #    self.pages_list_control.InsertItem(i, str(i))
            #self.pages_list_control.Select(self.activePage)

            self.pages_edit_list_control.SetStrings(self.pageNames)
            pages_list_control = self.pages_edit_list_control.GetListCtrl()
            pages_list_control.Select(self.activePage)
            pages_list_control.EnsureVisible(self.activePage)

    def On_MoveDn_Page(self, event):
        if self.slot_file_is_loaded == False:
            self.denied_message.ShowModal()
            return
        if self.activePage < len(self.motionPages) - 1:
            page = self.motionPages.pop(self.activePage)
            name = self.pageNames.pop(self.activePage)
            self.activePage += 1
            self.motionPages.insert(self.activePage, page)
            self.pageNames.insert(self.activePage, name)
            #self.pages_list_control.DeleteAllItems()
            #for i in range(len(self.motionPages)):
            #    self.pages_list_control.InsertItem(i, str(i))
            #self.pages_list_control.Select(self.activePage)

            self.pages_edit_list_control.SetStrings(self.pageNames)
            pages_list_control = self.pages_edit_list_control.GetListCtrl()
            pages_list_control.Select(self.activePage)
            pages_list_control.EnsureVisible(self.activePage)

    def On_Clone_Page(self, event):
        if self.slot_file_is_loaded == False:
            self.denied_message.ShowModal()
            return
        clone_page = self.motionPages[self.activePage].copy()
        page_name = self.pageNames[self.activePage]
        pos1 = page_name.find('clone')
        if pos1 > 0:
            try:
                number = int(page_name[pos1+5:])
            except Exception:
                number = 0
            clone_page_name = page_name[:pos1+5] + ' ' +str(number + 1)
        else:
            clone_page_name = page_name + ' clone 1'
        self.activePage += 1
        self.motionPages.insert(self.activePage, clone_page)
        self.pageNames.insert(self.activePage, clone_page_name)
        #self.pages_list_control.DeleteAllItems()
        #for i in range(len(self.motionPages)):
        #    self.pages_list_control.InsertItem(i, str(i))
        #self.pages_list_control.Select(self.activePage)
        
        self.pages_edit_list_control.SetStrings(self.pageNames)
        pages_list_control = self.pages_edit_list_control.GetListCtrl()
        pages_list_control.Select(self.activePage)
        pages_list_control.EnsureVisible(self.activePage)


    def On_Record_Page(self, event):
        if self.slot_file_is_loaded == False:
            self.denied_message.ShowModal()
            return
        self.motionPages[self.activePage][0] = self.activeFrames
        for i in range(self.dof):
            try:
                self.motionPages[self.activePage][i+1] = int(self.activePose[i])
            except Exception:
                self.motionPages[self.activePage].append(int(self.activePose[i]))
        print('recorded:', self.motionPages[self.activePage])

    #def On_Pages_Motion(self, event):
    #    item, flags = self.pages_list_control.HitTest(event.GetPosition())
    #    if item >= 0:
    #        self.pages_list_control.Select(item)
    #        self.curitem = item

    def On_Delete_Page(self, event):
        #pages_list_control = self.pages_edit_list_control.GetListCtrl()
        #item = pages_list_control.GetFirstSelected()
        if self.slot_file_is_loaded == False:
            self.denied_message.ShowModal()
            return
        if len(self.motionPages) > 1:
            self.motionPages.pop(self.activePage)
            self.pageNames.pop(self.activePage)
            self.activePage -= 1
            #self.pages_list_control.DeleteAllItems()
            #for i in range(len(self.motionPages)):
            #    self.pages_list_control.InsertItem(i, str(i))
            #self.pages_list_control.Select(self.activePage)
            self.pages_edit_list_control.SetStrings(self.pageNames)
            pages_list_control = self.pages_edit_list_control.GetListCtrl()
            pages_list_control.Select(self.activePage)
            pages_list_control.EnsureVisible(self.activePage)

    def On_Label_edit(self, event):
        pages_list_control = self.pages_edit_list_control.GetListCtrl()
        item = pages_list_control.GetFirstSelected()
        new_name = pages_list_control.GetItemText(item)
        print(new_name)
        self.pageNames[item] = new_name
        self.pages_edit_list_control.SetStrings(self.pageNames)
        pages_list_control = self.pages_edit_list_control.GetListCtrl()
        pages_list_control.Select(self.activePage)


    def On_Left_Down_edit(self, event):
        if self.slot_file_is_loaded == False:
            self.denied_message.ShowModal()
            return
        pages_list_control = self.pages_edit_list_control.GetListCtrl()
        #item = pages_list_control.GetFirstSelected()
        #if event.GetEventType() == wx.EVT_LIST_ITEM_ACTIVATED:
        #    item = pages_list_control.GetFocusedItem()
        #elif event.GetEventType() == wx.EVT_LIST_ITEM_FOCUSED:
        item = pages_list_control.GetFocusedItem()
        if 0 <= item < len(self.pageNames):
            print(pages_list_control.GetItemText(item))
            self.activePage = item
            print('self.activePage:', self.activePage)
            if len(self.motionPages) != 0:
                self.activeFrames = self.motionPages[self.activePage][0]
                self.activePoseOld = self.activePose.copy()
                for i in range(1, len(self.motionPages[self.activePage])):
                    self.activePose[i-1] = self.motionPages[self.activePage][i]
            self.refresh_Control_Values()
            pages_list_control = self.pages_edit_list_control.GetListCtrl()
            pages_list_control.Select(self.activePage)
            pages_list_control.EnsureVisible(self.activePage)

            #self.pages_list_control.Select(self.activePage)
            #self.pages_list_control.EnsureVisible(self.activePage)

    #def On_Left_Down_Pages(self, event):
    #    item, flags = self.pages_list_control.HitTest(event.GetPosition())
    #    #item = event.GetIndex()
    #    if item >= 0:
    #        self.pages_list_control.Select(item)
    #        value = item
    #        print(value, self.pages_list_control.GetItemText(value))
    #        self.activePage = value
    #        if self.pages_list_control.GetItemText(value) == 'trims':
    #            self.activePose = self.trims.copy()
    #            self.activePoseOld = self.activePose.copy()
    #            self.activeFrames = 1
    #        else:
    #            self.activePage = int(value)
    #            print('self.activePage:', self.activePage)
    #            if len(self.motionPages) != 0:
    #                self.activeFrames = self.motionPages[self.activePage][0]
    #                self.activePoseOld = self.activePose.copy()
    #                for i in range(1, len(self.motionPages[self.activePage])):
    #                    self.activePose[i-1] = self.motionPages[self.activePage][i]
    #        self.refresh_Control_Values()
    #        pages_list_control = self.pages_edit_list_control.GetListCtrl()
    #        pages_list_control.Select(self.activePage)
    #        pages_list_control.EnsureVisible(self.activePage)
    #        print('self.activeFrames:', self.activeFrames)
    #        print('self.activePose', self.activePose)



    def On_Physics(self, event):
        if event.GetEventObject() == self.physics_button:
            if self.physicsOn:
                self.physics_button.SetValue(False)
                self.physicsOn = False
                if SIMULATION == 1 or SIMULATION  == 0:
                    sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot)
                print('self.physicsOn = False')
            else:
                self.physics_button.SetValue(True)
                self.physicsOn = True
                if SIMULATION == 1 or SIMULATION  == 0:
                    sim.simxStartSimulation(self.clientID, sim.simx_opmode_oneshot)
                print('self.physicsOn = True')


    def Quick_Start(self, event):
        print('1. Здесь можно дать последовательность действий оператора',
              '\n2. Это должно помочь')


    def OnAbout(self, event):
        aboutInfo = wx.adv.AboutDialogInfo()
        aboutInfo.SetName("Pose Designer")
        aboutInfo.SetVersion('Version 1.0')
        aboutInfo.SetDescription("With this app you can tune fast and convenient\n   Robot pose and motion slots .")
        aboutInfo.SetCopyright("(C) 2020")
        aboutInfo.SetWebSite("www.robokit.su")
        aboutInfo.AddDeveloper("Azer Babaev")
        wx.adv.AboutBox(aboutInfo)

  
    def On_number_input(self, event):
        if event.GetEventObject() == self.frames_input:
            self.activeFrames = self.frames_input.GetValue()
        if event.GetEventObject() == self.slow_input:
            self.slow = self.slow_input.GetValue()

    def On_New_Slot(self, event):
        #slot_name_input = wx.Dialog(self, title='Slot name input')
        #dialog_sizer = wx.BoxSizer(wx.VERTICAL)
        #button_sizer = slot_name_input.CreateButtonSizer(flags =  wx.OK|wx.CANCEL)
        #text1 = wx.StaticText(slot_name_input, label='Input New Slot Name:',size=(200,25))
        radom_slot_number = "Motion_slot_Random_" + str(int( random.random() * 1000))
        slot_name_input = wx.TextEntryDialog(self, 'Slot name input', value=radom_slot_number)
        if slot_name_input.ShowModal() == wx.ID_OK:
            self.slot_name = slot_name_input.GetValue()
            print('self.slot_name = ', self.slot_name)
            title = 'Pose Designer ' + 'Motion Slot: ' + self.slot_name
            self.SetTitle(title)
            self.motionPages = [[ 0 for i in range(self.dof + 1)]]
            self.motionPages[0][0] = 1
            self.pageNames = ['page 0']
            print('self.motionPages:', self.motionPages)
            self.activePage = 0
            self.pages_edit_list_control.SetStrings(self.pageNames)
            pages_list_control = self.pages_edit_list_control.GetListCtrl()
            pages_list_control.Select(self.activePage)
            pages_list_control.EnsureVisible(self.activePage)

            #self.pages_list_control.DeleteAllItems()
            #self.pages_list_control.InsertItem(0, "0")
            
            #self.pages_list_control.Select(self.activePage)
            self.slot_file_is_loaded = True
            self.new_file_is_Not_saved = True


    def On_Save(self, event):
        print('save button pressed')
        if self.new_file_is_Not_saved :
            self.On_Save_as(event)
        else:
            slot_dict = {self.slot_name: self.motionPages, "pageNames": self.pageNames}
            with open(self.filename, "w") as f:
                    json.dump(slot_dict, f)

    def On_Save_as(self, event):
        slot_name_input = wx.TextEntryDialog(self, 'Final Slot name?', value=self.slot_name)
        if slot_name_input.ShowModal() == wx.ID_OK:
            self.slot_name = slot_name_input.GetValue()
            print('self.slot_name = ', self.slot_name)
            title = 'Pose Designer ' + 'Motion Slot: ' + self.slot_name
            self.SetTitle(title)
        defaultFile = self.slot_name + '.json' 
        save_file_dialog = wx.FileDialog(None, message="Select .json file tobe saved", defaultFile = defaultFile,
                                        wildcard = '*.json', style =wx.FD_SAVE|wx.FD_OVERWRITE_PROMPT)
        success_code = save_file_dialog.ShowModal()
        if success_code == wx.ID_OK:
            self.filename = save_file_dialog.GetPath()
            self.filename = self.filename.replace('\\', '/')
            slot_dict = {self.slot_name: self.motionPages, "pageNames": self.pageNames}
            with open(self.filename, "w") as f:
                json.dump(slot_dict, f)
            if self.new_file_is_Not_saved : self.new_file_is_Not_saved = False

    def On_Save_and_Exit(self, event):
        self.On_Save(event)
        self.On_Quit_select(event)

    def CreateMenuBar(self):

        menubar = wx.MenuBar()
        self.filem = wx.Menu()
        #reset = wx.Menu()
        #color_devices = wx.Menu()
        help = wx.Menu()
        self.calc = wx.Menu()
        self.utility = wx.Menu()
        #self.blobs = wx.Menu()

        #connect = wx.Menu()
        #connect.AppendRadioItem(6, 'Default', 'Connect through USB')
        #connect.AppendRadioItem(7, 'USB', 'Connect through USB')
        #connect.AppendRadioItem(8, 'WiFi', 'Connect through WiFi')

        self.filem.Append(100, '&New Slot', 'Create new motion slot from file')
        self.filem.Append(101, '&Load from file', 'Load motion slot from file')
        self.filem.Append(102, '&Save', 'Save motion slot to loaded file')
        self.filem.Append(103, 'Save as', 'Save motion slot to new file')
        self.filem.Append(105, '&Quit', 'Quit application')

        menubar.Append(self.filem, '&File')
        menubar.Append(help, '&Help')
        menubar.Append(self.calc, 'Calc')
        menubar.Append(self.utility, 'Utility')

        self.calc.Append(123, 'IK Calc', 'Inverse Kinematic')
        self.utility.Append(124, 'Mirror Transform', 'Mirror Transform')
        self.utility.Append(125, "ListBox Test", "ListBox Test")
        self.utility.Append(126, "Rename All Pages", "Rename All Pages to 'page XX'")
        self.utility.Append(127, "Dummy Report", "Monitor Dummy values")

        help.Append(121,'About')
        help.Append(122,'Quick Start')


        self.SetMenuBar(menubar)

    #def On_page_selector(self,event):
    #    if event.GetEventObject() == self.page_selector:
    #        value = self.page_selector.GetValue()
    #        if value == 'start':
    #            self.activePose = self.trims.copy()
    #            self.activePoseOld = self.activePose.copy()
    #            self.activeFrames = 1
    #        else:
    #            self.activePage = int(value)
    #            print('self.activePage:', self.activePage)
    #            if len(self.motionPages) != 0:
    #                self.activeFrames = self.motionPages[self.activePage][0]
    #                self.activePoseOld = self.activePose.copy()
    #                for i in range(1, len(self.motionPages[self.activePage])):
    #                    self.activePose[i-1] = self.motionPages[self.activePage][i]
    #        self.refresh_Control_Values()
    #        print('self.activeFrames:', self.activeFrames)
    #        print('self.activePose', self.activePose)

    def On_Slider_move(self, event):
        id = event.GetId()
        val = self.jointControls[id][1].GetValue()
        self.jointControls[id][2].SetValue(val)
        self.activePose[id] =  val 
        self.action_To_Robot()


    def On_SPINCTRL_change(self, event):
        id = event.GetId()
        if 0 <= id < self.dof:
            val = self.jointControls[id][2].GetValue()
            self.jointControls[id][1].SetValue(val)
            self.activePose[id] =  val
            self.action_To_Robot()

        

    def On_Quit_select(self, e):
        self.config = {'defaultFile': self.defaultFile}
        with open(current_work_directory + "pose_designer_config.json", "w") as f:
                json.dump(self.config, f)

        t = str(datetime.datetime.now())
        t = t.replace(" ", "_")
        t = t.replace(":", "_")
        t = t[:16]
        logfilename = current_work_directory + 'pose_designer_log/' + t + '.txt'
        self.log.SaveFile(logfilename)
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__
        sys.exit(0)

    def On_Syncro_Set(self, e):
        if self.syncro: 
            self.syncro = False
            self.syncro_button.SetLabel("NOSYNC")
        else: 
            self.syncro = True
            self.syncro_button.SetLabel ("SYNC")
            self.action_To_Robot()


    def On_Play_Page(self, event):
        if event.GetEventObject() == self.play_page_button:
            print('Play button pressed')
            if self.syncro:
                if SIMULATION == 1 or SIMULATION  == 0:
                    if self.physicsOn:
                        for k in range(self.activeFrames):
                            for j in range(self.dof):
                                tempActivePose = self.activePoseOld[j]+(self.activePose[j]-self.activePoseOld[j])*k/self.activeFrames
                                returnCode = sim.simxSetJointTargetPosition(self.clientID, self.jointHandle[j] ,
                                         (tempActivePose + self.trims[j]) * self.TIK2RAD * self.ACTIVESERVOS[j][3], sim.simx_opmode_oneshot)
                            sim.simxSynchronousTrigger(self.clientID)
                            time.sleep(self.slow / 1000)
                    else:
                        for j in range(self.dof):
                            returnCode = sim.simxSetJointPosition(self.clientID, self.jointHandle[j] ,
                                        (self.activePose[j] + self.trims[j]) * self.TIK2RAD * self.ACTIVESERVOS[j][3], sim.simx_opmode_oneshot)
                        time.sleep(self.slow / 1000)
                if SIMULATION == 5:
                    servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(self.dof + 2)]
                    for i in range(self.dof):
                        if self.ACTIVESERVOS[i][0] == 8:
                            n = self.dof - 1 + self.ACTIVESERVOS[i][1]
                            pos = int((self.activePose[i] + self.trims[i]) * self.ACTIVESERVOS[i][2]/2 + 7500)
                            servoDatas[n].Id, servoDatas[n].Sio, servoDatas[n].Data = 13, self.ACTIVESERVOS[i][1], pos
                        else: pos = int((self.activePose[i] + self.trims[i]) * self.ACTIVESERVOS[i][2] + 7500)
                        servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    a=self.rcb.setServoPosAsync(servoDatas, self.activeFrames, self.activeFrames-1)
                    time.sleep(self.slow / 1000)

    def On_Play_Next(self, event):
        print('PlayNext button pressed')
        if self.syncro:
            if self.activePage < len(self.motionPages) - 1:
                self.activePoseOld = self.activePose.copy()
                self.activePage += 1
                self.activeFrames = self.motionPages[self.activePage][0]
                for i in range(1, len(self.motionPages[self.activePage])):
                    self.activePose[i-1] = self.motionPages[self.activePage][i]
                if SIMULATION == 1 or SIMULATION  == 0:
                    if self.physicsOn:
                        for k in range(self.activeFrames):
                            for j in range(self.dof):
                                tempActivePose = self.activePoseOld[j]+(self.activePose[j]-self.activePoseOld[j])*k/self.activeFrames
                                returnCode = sim.simxSetJointTargetPosition(self.clientID, self.jointHandle[j] ,
                                            (tempActivePose + self.trims[j]) * self.TIK2RAD * self.ACTIVESERVOS[j][3], sim.simx_opmode_oneshot)
                            sim.simxSynchronousTrigger(self.clientID)
                            time.sleep(self.slow / 1000)
                    else:
                        for j in range(self.dof):
                            returnCode = sim.simxSetJointPosition(self.clientID, self.jointHandle[j] ,
                                        (self.activePose[j] + self.trims[j]) * self.TIK2RAD * self.ACTIVESERVOS[j][3], sim.simx_opmode_oneshot)
                        time.sleep(self.slow / 1000)
                if SIMULATION == 5:
                    servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(self.dof + 2)]
                    for i in range(self.dof):
                        if self.ACTIVESERVOS[i][0] == 8:
                            n = self.dof - 1 + self.ACTIVESERVOS[i][1]
                            pos = int((self.activePose[i] + self.trims[i]) * self.ACTIVESERVOS[i][2]/2 + 7500)
                            servoDatas[n].Id, servoDatas[n].Sio, servoDatas[n].Data = 13, self.ACTIVESERVOS[i][1], pos
                        else: pos = int((self.activePose[i] + self.trims[i]) * self.ACTIVESERVOS[i][2] + 7500)
                        servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    a=self.rcb.setServoPosAsync(servoDatas, self.activeFrames, self.activeFrames-1)
                    time.sleep(self.slow / 1000)
        pages_list_control = self.pages_edit_list_control.GetListCtrl()
        pages_list_control.Select(self.activePage)
        pages_list_control.EnsureVisible(self.activePage)
        #self.pages_list_control.Select(self.activePage)
        #self.pages_list_control.EnsureVisible(self.activePage)
        self.refresh_Control_Values()

    def On_Play_All(self, event):
        if event.GetEventObject() == self.play_all_button:
            print('PlayAll button pressed')
            if self.syncro:
                #value = self.page_selector.GetValue()
                #if value == 'start': self.activePage = 0
                if len(self.motionPages) != 0:
                    for page in range(self.activePage, len(self.motionPages)):
                        self.activeFrames = self.motionPages[page][0]
                        self.activePoseOld = self.activePose.copy()
                        for i in range(1, len(self.motionPages[page])):
                            self.activePose[i-1] = self.motionPages[page][i]
                        if SIMULATION == 1 or SIMULATION  == 0:
                            if self.physicsOn:
                                for k in range(self.activeFrames):
                                    for j in range(self.dof):
                                        tempActivePose = self.activePoseOld[j]+(self.activePose[j]-self.activePoseOld[j])*k/self.activeFrames
                                        returnCode = sim.simxSetJointTargetPosition(self.clientID, self.jointHandle[j] ,
                                                 (tempActivePose + self.trims[j]) * self.TIK2RAD * self.ACTIVESERVOS[j][3], sim.simx_opmode_oneshot)
                                    sim.simxSynchronousTrigger(self.clientID)
                                    time.sleep(self.slow / 1000)
                            else:
                                for j in range(self.dof):
                                    returnCode = sim.simxSetJointPosition(self.clientID, self.jointHandle[j] ,
                                                (self.activePose[j] + self.trims[j]) * self.TIK2RAD * self.ACTIVESERVOS[j][3], sim.simx_opmode_oneshot)
                                time.sleep(self.slow / 1000)
                        if SIMULATION == 5:
                            servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(self.dof + 2)]
                            for i in range(self.dof):
                                if self.ACTIVESERVOS[i][0] == 8:
                                    n = self.dof - 1 + self.ACTIVESERVOS[i][1]
                                    pos = int((self.activePose[i] + self.trims[i]) * self.ACTIVESERVOS[i][2]/2 + 7500)
                                    servoDatas[n].Id, servoDatas[n].Sio, servoDatas[n].Data = 13, self.ACTIVESERVOS[i][1], pos
                                else: pos = int((self.activePose[i] + self.trims[i]) * self.ACTIVESERVOS[i][2] + 7500)
                                servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                            a=self.rcb.setServoPosAsync(servoDatas, self.activeFrames, self.activeFrames-1)
                            time.sleep(self.slow / 1000)
                        self.activePage = page
                        #self.page_selector.SetValue(str(self.activePage))
                        #self.pages_list_control.Select(self.activePage)
                        #self.pages_list_control.EnsureVisible(self.activePage)

                        pages_list_control = self.pages_edit_list_control.GetListCtrl()
                        pages_list_control.Select(self.activePage)
                        pages_list_control.EnsureVisible(self.activePage)
                        self.refresh_Control_Values()

    def On_Return(self, event):
        if event.GetEventObject() == self.return_button:
            print('Ruturn button pressed')
            if self.syncro:
                if SIMULATION == 1 or SIMULATION  == 0:
                    if self.physicsOn:
                        for k in range(self.activeFrames):
                            for j in range(self.dof):
                                tempActivePose = self.activePoseOld[j]+(self.activePose[j]-self.activePoseOld[j])*(self.activeFrames - k)/self.activeFrames
                                returnCode = sim.simxSetJointTargetPosition(self.clientID, self.jointHandle[j] ,
                                         (tempActivePose + self.trims[j]) * self.TIK2RAD * self.ACTIVESERVOS[j][3], sim.simx_opmode_oneshot)
                            sim.simxSynchronousTrigger(self.clientID)
                    else:
                        for j in range(self.dof):
                            returnCode = sim.simxSetJointPosition(self.clientID, self.jointHandle[j] ,
                                        (self.activePoseOld[j] + self.trims[j]) * self.TIK2RAD * self.ACTIVESERVOS[j][3], sim.simx_opmode_oneshot)
                if SIMULATION == 5:
                    servoDatas = [self.Roki.Rcb4.ServoData() for _ in range(self.dof + 2)]
                    for i in range(self.dof):
                        if self.ACTIVESERVOS[i][0] == 8:
                            n = self.dof - 1 + self.ACTIVESERVOS[i][1]
                            pos = int((self.activePoseOld[i]+ self.trims[i]) * self.ACTIVESERVOS[i][2]/2 + 7500)
                            servoDatas[n].Id, servoDatas[n].Sio, servoDatas[n].Data = 13, self.ACTIVESERVOS[i][1], pos
                        else: pos = int((self.activePoseOld[i]+ self.trims[i]) * self.ACTIVESERVOS[i][2] + 7500)
                        servoDatas[i].Id, servoDatas[i].Sio, servoDatas[i].Data = self.ACTIVESERVOS[i][0], self.ACTIVESERVOS[i][1], pos
                    a=self.rcb.setServoPosAsync(servoDatas, self.activeFrames, self.activeFrames-1)
                    time.sleep(self.slow / 1000)

    def On_Load_File(self, event):
        print('On_Load_File')
        load_file_dialog = wx.FileDialog(None, message="Select .json file with motion slot", defaultFile = self.defaultFile, wildcard = '*.json')
        print('FileDialog created')
        success_code = load_file_dialog.ShowModal()
        print('File selected')
        if success_code == wx.ID_OK:
            self.filename = load_file_dialog.GetPath()
            self.defaultFile = self.filename
            self.filename = self.filename.replace('\\', '/')
            with open(self.filename, "r") as f:
                loaded_Dict = json.loads(f.read())
            print(loaded_Dict.keys())
            self.slot_name = str(list(loaded_Dict.keys())[0])
            self.motionPages = loaded_Dict[self.slot_name]
            if loaded_Dict.get("pageNames") != None:
                self.pageNames = loaded_Dict["pageNames"]
            else:
                self.pageNames = []
                for i in range(len(self.motionPages)):
                    pageName = 'page '+ str(i)
                    self.pageNames.append(pageName) 
            title = 'Pose Designer ' + 'Motion Slot: ' + self.slot_name
            self.SetTitle(title)
            print(self.motionPages)
            #for i in range(len(self.motionPages)):
            #    self.page_selector.SetString(i+1,str(i))
            #print('GetCount:',self.page_selector.GetCount())
            self.activePage = 0
            self.pages_edit_list_control.SetStrings(self.pageNames)
            pages_list_control = self.pages_edit_list_control.GetListCtrl()
            pages_list_control.Select(self.activePage)
            pages_list_control.EnsureVisible(self.activePage)
            #self.pages_list_control.DeleteAllItems()
            #for i in range(len(self.motionPages)):
            #    self.pages_list_control.InsertItem(i, str(i))
            
            #self.pages_list_control.Select(self.activePage)
            self.slot_file_is_loaded = True
        print( 'slot_file_is_loaded =', self.slot_file_is_loaded)

           
        
def sim_Enable(ip_address, port):
    simThreadCycleInMs = 5
    print ('Simulation started')
    #sim.simxFinish(-1) # just in case, close all opened connections
    clientID = sim.simxStart(ip_address, port, True, True, 5000, simThreadCycleInMs)
    if clientID != -1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')
        print ('Program ended')
        exit(0)
    return clientID

def main():
    app = wx.App()
    pd = Pose_Designer(None)
    pd.Show()
    app.MainLoop()


if __name__ == '__main__':
    main()  
