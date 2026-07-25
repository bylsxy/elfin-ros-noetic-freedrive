#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 28 12:18:05 2017

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2017, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 
"""
# author: Cong Liu

from __future__ import division
import rospy
import math
import time
import tf
import moveit_commander
import rosservice
from std_msgs.msg import Bool, Float64, Float64MultiArray, String, UInt32
from sensor_msgs.msg import JointState
from std_srvs.srv import (SetBool, SetBoolRequest, SetBoolResponse,
                          Trigger, TriggerRequest)
from elfin_robot_msgs.srv import SetString, SetStringRequest, SetStringResponse
from elfin_robot_msgs.srv import SetInt16, SetInt16Request
from elfin_robot_msgs.srv import SetFloat64, SetFloat64Request
from elfin_freedrive_controller.srv import (
    SetDampingScales, SetDampingScalesRequest)
from elfin_robot_msgs.srv import *
import wx
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import threading
import dynamic_reconfigure.client

class MyFrame(wx.Frame):  

    POINT_DI_BIT=4
    FREE_DI_BIT=5
    def __init__(self,parent,id):  
        wx.Frame.__init__(self,parent,id,'Elfin E05 新手控制面板',pos=(120,60))
        self.panel=wx.ScrolledWindow(
            self, style=wx.HSCROLL | wx.VSCROLL | wx.TAB_TRAVERSAL)
        self.panel.SetScrollRate(12, 12)
        font=self.panel.GetFont()
        font.SetPointSize(max(font.GetPointSize(), 10))
        self.panel.SetFont(font)
        self.main_sizer=wx.BoxSizer(wx.VERTICAL)
        frame_sizer=wx.BoxSizer(wx.VERTICAL)
        frame_sizer.Add(self.panel, 1, wx.EXPAND)
        self.SetSizer(frame_sizer)
        
        self.listener = tf.TransformListener()
        
        self.robot=moveit_commander.RobotCommander()
        self.scene=moveit_commander.PlanningSceneInterface()
        self.group=moveit_commander.MoveGroupCommander('elfin_arm')

        self.controller_ns='elfin_arm_controller/'
        self.elfin_driver_ns='elfin_ros_control/elfin/'
        self.elfin_IO_ns='elfin_ros_control/elfin/io_port1/' # 20201126: add IO ns

        self.call_read_do_req = ElfinIODReadRequest()
        self.call_read_di_req = ElfinIODReadRequest()
        self.call_read_do_req.data = True
        self.call_read_di_req.data = True
        self.call_read_do = rospy.ServiceProxy(self.elfin_IO_ns+'read_do',ElfinIODRead)
        self.call_read_di = rospy.ServiceProxy(self.elfin_IO_ns+'read_di',ElfinIODRead)
        # 20201126: add service for write_do
        self.call_write_DO=rospy.ServiceProxy(self.elfin_IO_ns+'write_do',ElfinIODWrite)
        
        self.elfin_basic_api_ns='elfin_basic_api/'
        
        self.joint_names=rospy.get_param(self.controller_ns+'joints', [])
        
        self.ref_link_name=self.group.get_planning_frame()
        self.end_link_name=self.group.get_end_effector_link()
        
        self.ref_link_lock=threading.Lock()
        self.end_link_lock=threading.Lock()
        self.DO_btn_lock = threading.Lock() # 20201208: add the threading lock
        self.DI_show_lock = threading.Lock()
                
        self.js_display=[0]*6 # joint_states
        self.jm_button=[0]*6 # joints_minus
        self.jp_button=[0]*6 # joints_plus
        self.js_label=[0]*6 # joint_states
                      
        self.ps_display=[0]*6 # pcs_states
        self.pm_button=[0]*6 # pcs_minus
        self.pp_button=[0]*6 # pcs_plus
        self.ps_label=[0]*6 # pcs_states

        # The E05 electrical manual defines exactly three digital inputs and
        # three digital outputs.  The old four-DI/four-DO/four-LED layout came
        # from a different end-I/O revision and must not be presented as E05.
        self.DO_btn_display=[0]*3
        self.DI_display=[0]*3
        self.DO_btn=[0]*3
        self.DI_show=[0]*16
        self.di_raw_value=0
        self.di_seen=False
        self.io_online=False
        self.io_poll_failures=0
        self.io_retry_after=0.0
        self.last_io_error=''

        # This E05's physical tool buttons have been verified against the raw
        # 16-bit read_di word.  These are not the external INPUT_0..2 terminals.
        self.tool_button_lock=threading.Lock()
        self.tool_button_bits={
            'POINT': self.POINT_DI_BIT,
            'FREE': self.FREE_DI_BIT}
        self.tool_button_pressed={'POINT': False, 'FREE': False}

        self.freedrive_state='STARTING'
        self.freedrive_detail='等待 elfin_freedrive_manager'
        self.freedrive_active=False
        self.freedrive_ring_state='UNKNOWN_RING_STATE'
        self.freedrive_point_count=0
        self.freedrive_validation='等待重力模型预检'
        self.freedrive_trial_log='尚未开始本次试验'
        self.freedrive_velocity_scale=1.0
        self.freedrive_velocity_hard_limits=[]
        self.freedrive_damping_scales=[1.0]*6
        self.freedrive_state_lock=threading.Lock()

        self.key=[]

        self.create_main_controls()
        self.display_init()

        self.servo_state=bool()
        self.servo_state_received=False
        self.servo_state_lock=threading.Lock()

        self.fault_state=bool()
        self.fault_state_received=False
        self.fault_state_lock=threading.Lock()

        self.teleop_api_dynamic_reconfig_client=dynamic_reconfigure.client.Client(self.elfin_basic_api_ns,
                                                                                  config_callback=self.basic_api_reconfigure_cb)
        
        self.dlg=wx.Dialog(self.panel, title='机器人操作进度',
                           style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
        self.dlg.Bind(wx.EVT_CLOSE, self.closewindow)
        self.dlg_label=wx.StaticText(self.dlg, label='hello')
        self.dlg_sizer=wx.BoxSizer(wx.VERTICAL)
        self.dlg_sizer.Add(self.dlg_label, 0, wx.ALL | wx.EXPAND, 15)
        self.dlg.SetSizer(self.dlg_sizer)
        self.dlg_sizer.Fit(self.dlg)
        
        self.set_links_dlg=wx.Dialog(self.panel, title='设置笛卡尔坐标系',
                                     style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
        
        self.sld_ref_link_show=wx.TextCtrl(self.set_links_dlg, style=wx.TE_PROCESS_ENTER,
                                           value='', size=(360, -1))
        self.sld_end_link_show=wx.TextCtrl(self.set_links_dlg, style=wx.TE_PROCESS_ENTER,
                                           value='', size=(360, -1))
        
        self.sld_set_ref_link_btn=wx.Button(self.set_links_dlg, label='更新参考坐标系',
                                            name='Update ref. link')
        self.sld_set_end_link_btn=wx.Button(self.set_links_dlg, label='更新末端坐标系',
                                            name='Update end link')
        links_sizer=wx.FlexGridSizer(rows=2, cols=3, vgap=10, hgap=10)
        links_sizer.Add(self.sld_ref_link_show, 1, wx.EXPAND)
        links_sizer.Add(self.sld_set_ref_link_btn, 0, wx.EXPAND)
        links_sizer.Add(self.make_description(
            self.set_links_dlg, '改变 X/Y/Z、Rx/Ry/Rz 点动所依据的参考坐标系。', 320),
            0, wx.ALIGN_CENTER_VERTICAL)
        links_sizer.Add(self.sld_end_link_show, 1, wx.EXPAND)
        links_sizer.Add(self.sld_set_end_link_btn, 0, wx.EXPAND)
        links_sizer.Add(self.make_description(
            self.set_links_dlg, '改变笛卡尔点动所控制的末端连杆；普通使用保持默认。', 320),
            0, wx.ALIGN_CENTER_VERTICAL)
        links_sizer.AddGrowableCol(0, 1)
        links_sizer.AddGrowableCol(2, 1)
        links_outer_sizer=wx.BoxSizer(wx.VERTICAL)
        links_outer_sizer.Add(links_sizer, 1, wx.ALL | wx.EXPAND, 15)
        self.set_links_dlg.SetSizerAndFit(links_outer_sizer)
        
                        
        self.call_teleop_joint=rospy.ServiceProxy(self.elfin_basic_api_ns+'joint_teleop', 
                                                  SetInt16)
        self.call_teleop_joint_req=SetInt16Request()
        
        self.call_teleop_cart=rospy.ServiceProxy(self.elfin_basic_api_ns+'cart_teleop', 
                                                 SetInt16)
        self.call_teleop_cart_req=SetInt16Request()
        
        self.call_teleop_stop=rospy.ServiceProxy(self.elfin_basic_api_ns+'stop_teleop', 
                                                 SetBool)
        self.call_teleop_stop_req=SetBoolRequest()
        
        self.call_stop=rospy.ServiceProxy(self.elfin_basic_api_ns+'stop_teleop', 
                                          SetBool)
        self.call_stop_req=SetBoolRequest()
        self.call_stop_req.data=True
        self.stop_btn.Bind(wx.EVT_BUTTON, 
                           lambda evt, cl=self.call_stop,
                           rq=self.call_stop_req :
                           self.call_set_bool_common(evt, cl, rq))
            
        self.call_reset=rospy.ServiceProxy(self.elfin_driver_ns+'clear_fault', SetBool)
        self.call_reset_req=SetBoolRequest()
        self.call_reset_req.data=True
        self.reset_btn.Bind(wx.EVT_BUTTON, 
                           lambda evt, cl=self.call_reset,
                           rq=self.call_reset_req :
                           self.call_set_bool_common(evt, cl, rq))
                
        self.call_power_on=rospy.ServiceProxy(self.elfin_basic_api_ns+'enable_robot', SetBool)
        self.call_power_on_req=SetBoolRequest()
        self.call_power_on_req.data=True
        self.power_on_btn.Bind(wx.EVT_BUTTON, 
                               lambda evt, cl=self.call_power_on,
                               rq=self.call_power_on_req :
                               self.call_set_bool_common(evt, cl, rq))
        
        self.call_power_off=rospy.ServiceProxy(self.elfin_basic_api_ns+'disable_robot', SetBool)
        self.call_power_off_req=SetBoolRequest()
        self.call_power_off_req.data=True
        self.power_off_btn.Bind(wx.EVT_BUTTON, 
                               lambda evt, cl=self.call_power_off,
                               rq=self.call_power_off_req :
                               self.call_set_bool_common(evt, cl, rq))

        self.call_set_freedrive=rospy.ServiceProxy(
            '/elfin_freedrive_manager/set_freedrive', SetBool)
        self.call_record_freedrive_point=rospy.ServiceProxy(
            '/elfin_freedrive_manager/record_point', Trigger)
        self.call_set_freedrive_velocity_scale=rospy.ServiceProxy(
            '/elfin_freedrive_manager/set_velocity_limit_scale', SetFloat64)
        self.call_set_freedrive_damping_scales=rospy.ServiceProxy(
            '/elfin_freedrive_manager/set_damping_scales', SetDampingScales)
        self.enter_freedrive_btn.Bind(
            wx.EVT_BUTTON, lambda evt: self.request_freedrive(evt, True))
        self.exit_freedrive_btn.Bind(
            wx.EVT_BUTTON, lambda evt: self.request_freedrive(evt, False))
        self.record_point_btn.Bind(wx.EVT_BUTTON, self.request_record_point)
        self.freedrive_speed_slider.Bind(
            wx.EVT_SLIDER, self.preview_freedrive_speed_scale)
        self.freedrive_speed_apply_btn.Bind(
            wx.EVT_BUTTON, self.request_freedrive_speed_scale)
        self.freedrive_damping_apply_btn.Bind(
            wx.EVT_BUTTON, self.request_freedrive_damping_scales)
                
        self.call_move_homing=rospy.ServiceProxy(self.elfin_basic_api_ns+'home_teleop', 
                                                 SetBool)
        self.call_move_homing_req=SetBoolRequest()
        self.call_move_homing_req.data=True
        self.home_btn.Bind(wx.EVT_LEFT_DOWN, 
                           lambda evt, cl=self.call_move_homing,
                           rq=self.call_move_homing_req :
                           self.call_set_bool_common(evt, cl, rq))
        self.home_btn.Bind(wx.EVT_LEFT_UP,
                           lambda evt, mark=100:
                           self.release_button(evt, mark) )
            
        self.call_set_ref_link=rospy.ServiceProxy(self.elfin_basic_api_ns+'set_reference_link', SetString)
        self.call_set_end_link=rospy.ServiceProxy(self.elfin_basic_api_ns+'set_end_link', SetString)
        self.set_links_btn.Bind(wx.EVT_BUTTON, self.show_set_links_dialog)
        self.maintenance_btn.Bind(wx.EVT_BUTTON, self.show_maintenance_dialog)
        
        self.sld_set_ref_link_btn.Bind(wx.EVT_BUTTON, self.update_ref_link)
        self.sld_set_end_link_btn.Bind(wx.EVT_BUTTON, self.update_end_link)
        
        self.sld_ref_link_show.Bind(wx.EVT_TEXT_ENTER, self.update_ref_link)
        self.sld_end_link_show.Bind(wx.EVT_TEXT_ENTER, self.update_end_link)
            
        self.action_client=SimpleActionClient(self.controller_ns+'follow_joint_trajectory',
                                              FollowJointTrajectoryAction)
        self.action_goal=FollowJointTrajectoryGoal()
        self.action_goal.trajectory.joint_names=self.joint_names

        self.init_maintenance_dialog()
        self.Bind(wx.EVT_CLOSE, self.on_main_close)

        self.panel.SetSizer(self.main_sizer)
        self.panel.Layout()
        self.panel.FitInside()
        display_size=wx.GetDisplaySize()
        display_width=display_size.GetWidth()
        display_height=display_size.GetHeight()
        frame_width=max(900, min(1220, display_width-80))
        frame_height=max(620, min(900, display_height-100))
        self.SetMinSize((min(900, frame_width), min(620, frame_height)))
        self.SetSize((frame_width, frame_height))
        self.Centre()

    def make_description(self, parent, text, width=220):
        label=wx.StaticText(parent, label=text)
        label.Wrap(width)
        label.SetForegroundColour(wx.Colour(70, 70, 70))
        return label

    def create_main_controls(self):
        title=wx.StaticText(self.panel, label='Elfin E05 人工控制与只读诊断')
        title_font=title.GetFont()
        title_font.SetPointSize(title_font.GetPointSize()+4)
        title_font.SetWeight(wx.FONTWEIGHT_BOLD)
        title.SetFont(title_font)
        self.main_sizer.Add(title, 0, wx.LEFT | wx.RIGHT | wx.TOP, 12)

        safety=self.make_description(
            self.panel,
            '运动按钮均为真实控制入口：先清场、从 1% 开始，并让一人守上游电闸。'
            '松开点动按钮会请求 Stop；Stop 不是物理断电。',
            820)
        safety.SetForegroundColour(wx.Colour(170, 45, 35))
        self.main_sizer.Add(safety, 0, wx.ALL | wx.EXPAND, 12)

        state_box=wx.StaticBoxSizer(wx.StaticBox(self.panel, label='机器人状态'), wx.VERTICAL)
        state_grid=wx.FlexGridSizer(rows=7, cols=3, vgap=8, hgap=10)
        state_grid.Add(wx.StaticText(self.panel, label='伺服状态'), 0, wx.ALIGN_CENTER_VERTICAL)
        self.servo_state_show=wx.TextCtrl(
            self.panel, style=wx.TE_CENTER | wx.TE_READONLY, value='等待状态...')
        state_grid.Add(self.servo_state_show, 0, wx.EXPAND)
        state_grid.Add(self.make_description(
            self.panel, '显示六轴是否已经 Servo On；它不代表机械臂一定无故障。', 420),
            0, wx.ALIGN_CENTER_VERTICAL)
        state_grid.Add(wx.StaticText(self.panel, label='故障状态'), 0, wx.ALIGN_CENTER_VERTICAL)
        self.fault_state_show=wx.TextCtrl(
            self.panel, style=wx.TE_CENTER | wx.TE_READONLY, value='等待状态...')
        state_grid.Add(self.fault_state_show, 0, wx.EXPAND)
        state_grid.Add(self.make_description(
            self.panel, 'Fault 表示底层驱动保护已触发；先排除机械接触或供电原因，再清故障。', 420),
            0, wx.ALIGN_CENTER_VERTICAL)
        state_grid.Add(wx.StaticText(self.panel, label='零力拖拽状态'),
                       0, wx.ALIGN_CENTER_VERTICAL)
        self.freedrive_state_show=wx.TextCtrl(
            self.panel, style=wx.TE_CENTER | wx.TE_READONLY,
            value='等待管理节点...')
        state_grid.Add(self.freedrive_state_show, 0, wx.EXPAND)
        state_grid.Add(self.make_description(
            self.panel,
            'READY 可进入；实体 FREE 新按下沿会立即请求；ACTIVE 表示重力补偿控制器正在占用六轴。',
            420), 0, wx.ALIGN_CENTER_VERTICAL)
        state_grid.Add(wx.StaticText(self.panel, label='拖拽状态详情'),
                       0, wx.ALIGN_CENTER_VERTICAL)
        self.freedrive_detail_show=wx.TextCtrl(
            self.panel,
            style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_WORDWRAP,
            value='等待 elfin_freedrive_manager')
        self.freedrive_detail_show.SetMinSize((-1, 52))
        state_grid.Add(self.freedrive_detail_show, 0, wx.EXPAND)
        state_grid.Add(self.make_description(
            self.panel,
            '这里显示拒绝原因、切换进度和保护回退结果；文本会自动换行，可滚动查看。',
            420), 0, wx.ALIGN_CENTER_VERTICAL)
        state_grid.Add(wx.StaticText(self.panel, label='末端灯环语义'),
                       0, wx.ALIGN_CENTER_VERTICAL)
        self.freedrive_ring_show=wx.TextCtrl(
            self.panel, style=wx.TE_CENTER | wx.TE_READONLY,
            value='未知（等待管理节点）')
        state_grid.Add(self.freedrive_ring_show, 0, wx.EXPAND)
        state_grid.Add(self.make_description(
            self.panel,
            '按手册显示应有颜色：绿=Servo On、黄=Servo Off、红=Fault、蓝=零力。当前驱动未提供主动指定灯色接口。',
            420), 0, wx.ALIGN_CENTER_VERTICAL)
        state_grid.Add(wx.StaticText(self.panel, label='重力模型预检'),
                       0, wx.ALIGN_CENTER_VERTICAL)
        self.freedrive_validation_show=wx.TextCtrl(
            self.panel,
            style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_WORDWRAP,
            value='等待重力模型预检')
        self.freedrive_validation_show.SetMinSize((-1, 48))
        state_grid.Add(self.freedrive_validation_show, 0, wx.EXPAND)
        state_grid.Add(self.make_description(
            self.panel,
            '持续比较位置模式保持力矩与模型。单姿态反馈只用于预检和无跳变接管，不会把人手预载学习成重力倍率。',
            420), 0, wx.ALIGN_CENTER_VERTICAL)
        state_grid.Add(wx.StaticText(self.panel, label='本次拖拽数据'),
                       0, wx.ALIGN_CENTER_VERTICAL)
        self.freedrive_trial_log_show=wx.TextCtrl(
            self.panel,
            style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_WORDWRAP,
            value='尚未开始本次试验')
        self.freedrive_trial_log_show.SetMinSize((-1, 48))
        state_grid.Add(self.freedrive_trial_log_show, 0, wx.EXPAND)
        state_grid.Add(self.make_description(
            self.panel,
            '自动 CSV 同步保存位置、速度、反馈力矩、模型力矩、下发力矩及退出原因，供逐轴标定和复现。',
            420), 0, wx.ALIGN_CENTER_VERTICAL)
        state_grid.AddGrowableCol(1, 1)
        state_grid.AddGrowableCol(2, 1)
        state_box.Add(state_grid, 0, wx.ALL | wx.EXPAND, 10)
        self.main_sizer.Add(state_box, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 12)

        command_box=wx.StaticBoxSizer(wx.StaticBox(self.panel, label='整机命令'), wx.VERTICAL)
        command_grid=wx.FlexGridSizer(rows=0, cols=2, vgap=10, hgap=12)
        command_defs=(
            ('power_on_btn', '伺服上电\nServo On', 'Servo On',
             '检查位置对齐后使能六轴并启动轨迹控制器。'),
            ('power_off_btn', '伺服关闭\nServo Off', 'Servo Off',
             '取消轨迹并关闭六轴伺服；正常停机优先使用。'),
            ('reset_btn', '清除故障\nClear Fault', 'Clear Fault',
             '仅清除故障锁存，不会消除碰撞、过载或供电原因。'),
            ('home_btn', '回 ROS 零位\nHome', 'home_btn',
             '按住才运动，松开停止；目标是固定标定零位，不是开机姿态。'),
            ('stop_btn', '停止轨迹\nStop', 'Stop',
             '取消当前点动/轨迹，但保持伺服状态；紧急情况仍应断电。'))
        for attribute, label, name, description in command_defs:
            column=wx.BoxSizer(wx.VERTICAL)
            button=wx.Button(self.panel, label=label, name=name)
            button.SetMinSize((180, 52))
            button.SetToolTip(description)
            setattr(self, attribute, button)
            column.Add(button, 0, wx.EXPAND | wx.BOTTOM, 5)
            column.Add(self.make_description(self.panel, description, 360), 1, wx.EXPAND)
            command_grid.Add(column, 1, wx.EXPAND)
        for column in range(2):
            command_grid.AddGrowableCol(column, 1)
        command_box.Add(command_grid, 0, wx.ALL | wx.EXPAND, 10)
        self.main_sizer.Add(command_box, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 12)

        velocity_scaling_init=rospy.get_param(
            self.elfin_basic_api_ns+'velocity_scaling', default=0.01)
        speed_box=wx.StaticBoxSizer(wx.StaticBox(self.panel, label='点动速度倍率'), wx.VERTICAL)
        speed_row=wx.BoxSizer(wx.HORIZONTAL)
        speed_row.Add(wx.StaticText(self.panel, label='1%'), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 8)
        self.velocity_setting=wx.Slider(
            self.panel, value=int(velocity_scaling_init*100), minValue=1, maxValue=100,
            style=wx.SL_HORIZONTAL)
        speed_row.Add(self.velocity_setting, 1, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 8)
        speed_row.Add(wx.StaticText(self.panel, label='100%'), 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 8)
        self.velocity_setting_show=wx.TextCtrl(
            self.panel, style=wx.TE_CENTER | wx.TE_READONLY,
            value=str(round(velocity_scaling_init*100, 1))+'%', size=(90, -1))
        speed_row.Add(self.velocity_setting_show, 0, wx.ALIGN_CENTER_VERTICAL)
        speed_box.Add(speed_row, 0, wx.ALL | wx.EXPAND, 10)
        speed_box.Add(self.make_description(
            self.panel,
            '只影响 Panel/Basic API 点动生成的轨迹；RViz MoveIt 有独立的速度与加速度倍率。',
            820), 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 10)
        self.velocity_setting.Bind(wx.EVT_SLIDER, self.velocity_setting_cb)
        self.main_sizer.Add(speed_box, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 12)

        link_box=wx.StaticBoxSizer(wx.StaticBox(self.panel, label='笛卡尔坐标与工具'), wx.VERTICAL)
        link_grid=wx.FlexGridSizer(rows=2, cols=3, vgap=8, hgap=10)
        link_grid.Add(wx.StaticText(self.panel, label='参考坐标系 Ref. link'),
                      0, wx.ALIGN_CENTER_VERTICAL)
        self.ref_link_show=wx.TextCtrl(
            self.panel, style=wx.TE_READONLY, value=self.ref_link_name)
        link_grid.Add(self.ref_link_show, 0, wx.EXPAND)
        self.set_links_btn=wx.Button(self.panel, label='设置坐标系', name='Set links')
        self.set_links_btn.SetToolTip('打开参考坐标系与末端连杆设置')
        link_grid.Add(self.set_links_btn, 0, wx.EXPAND)

        link_grid.Add(wx.StaticText(self.panel, label='末端连杆 End link'),
                      0, wx.ALIGN_CENTER_VERTICAL)
        self.end_link_show=wx.TextCtrl(
            self.panel, style=wx.TE_READONLY, value=self.end_link_name)
        link_grid.Add(self.end_link_show, 0, wx.EXPAND)
        self.maintenance_btn=wx.Button(
            self.panel, label='维护与接口诊断', name='Maintenance')
        self.maintenance_btn.SetToolTip('受保护抱闸控制与只读驱动诊断')
        link_grid.Add(self.maintenance_btn, 0, wx.EXPAND)
        link_grid.AddGrowableCol(1, 1)
        link_box.Add(link_grid, 0, wx.ALL | wx.EXPAND, 10)
        link_box.Add(self.make_description(
            self.panel,
            '“设置坐标系”只改变 X/Y/Z、Rx/Ry/Rz 的参考坐标系和末端连杆；'
            '“维护与接口诊断”包含受保护抱闸操作，进入后仍需逐项确认。', 820),
            0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 10)
        self.main_sizer.Add(link_box, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 12)

    def display_init(self):
        motion_column=wx.BoxSizer(wx.VERTICAL)
        joint_box=wx.StaticBoxSizer(wx.StaticBox(self.panel, label='关节点动'), wx.VERTICAL)
        joint_grid=wx.FlexGridSizer(rows=7, cols=5, vgap=7, hgap=8)
        for heading in ('关节', '正方向', '负方向', '实时角度', '中文说明'):
            label=wx.StaticText(self.panel, label=heading)
            label.SetFont(label.GetFont().Bold())
            joint_grid.Add(label, 0, wx.ALIGN_CENTER_VERTICAL)

        joint_descriptions=(
            'J1 基座旋转；方向以 ROS 正负号为准。',
            'J2 肩关节俯仰；会显著改变整臂高度。',
            'J3 肘关节折叠；会显著改变伸展距离。',
            'J4 腕部第一轴/前臂旋转。',
            'J5 腕部俯仰；注意末端与小臂自碰撞。',
            'J6 工具法兰旋转；注意末端线缆缠绕。')
        for i in range(6):
            self.js_label[i]=wx.StaticText(self.panel, label='J'+str(i+1))
            joint_grid.Add(self.js_label[i], 0, wx.ALIGN_CENTER_VERTICAL)
            self.jp_button[i]=wx.Button(self.panel, label='J'+str(i+1)+' +')
            self.jp_button[i].SetToolTip('按住：J'+str(i+1)+' 沿 ROS 正方向连续点动；松开：Stop')
            self.jp_button[i].Bind(
                wx.EVT_LEFT_DOWN, lambda evt, mark=i+1: self.teleop_joints(evt, mark))
            self.jp_button[i].Bind(
                wx.EVT_LEFT_UP, lambda evt, mark=i+1: self.release_button(evt, mark))
            joint_grid.Add(self.jp_button[i], 0, wx.EXPAND)
            self.jm_button[i]=wx.Button(self.panel, label='J'+str(i+1)+' -')
            self.jm_button[i].SetToolTip('按住：J'+str(i+1)+' 沿 ROS 负方向连续点动；松开：Stop')
            self.jm_button[i].Bind(
                wx.EVT_LEFT_DOWN, lambda evt, mark=-1*(i+1): self.teleop_joints(evt, mark))
            self.jm_button[i].Bind(
                wx.EVT_LEFT_UP, lambda evt, mark=-1*(i+1): self.release_button(evt, mark))
            joint_grid.Add(self.jm_button[i], 0, wx.EXPAND)
            self.js_display[i]=wx.TextCtrl(
                self.panel, style=wx.TE_CENTER | wx.TE_READONLY,
                value='--', size=(95, -1))
            joint_grid.Add(self.js_display[i], 0, wx.EXPAND | wx.ALIGN_CENTER_VERTICAL)
            joint_grid.Add(self.make_description(
                self.panel, joint_descriptions[i]+' 两个按钮都需按住，松开即停。', 250),
                0, wx.ALIGN_CENTER_VERTICAL)
        joint_grid.AddGrowableCol(4, 1)
        joint_box.Add(joint_grid, 0, wx.ALL | wx.EXPAND, 10)
        motion_column.Add(joint_box, 0, wx.BOTTOM | wx.EXPAND, 10)

        cart_box=wx.StaticBoxSizer(wx.StaticBox(self.panel, label='笛卡尔末端点动'), wx.VERTICAL)
        cart_grid=wx.FlexGridSizer(rows=7, cols=5, vgap=7, hgap=8)
        for heading in ('轴', '正方向', '负方向', '实时值', '中文说明'):
            label=wx.StaticText(self.panel, label=heading)
            label.SetFont(label.GetFont().Bold())
            cart_grid.Add(label, 0, wx.ALIGN_CENTER_VERTICAL)

        cart_names=('X', 'Y', 'Z', 'Rx', 'Ry', 'Rz')
        cart_display_names=('X / mm', 'Y / mm', 'Z / mm', 'R / deg', 'P / deg', 'Y / deg')
        cart_descriptions=(
            '沿参考坐标系 X 轴平移末端。',
            '沿参考坐标系 Y 轴平移末端。',
            '沿参考坐标系 Z 轴平移末端。',
            '绕参考坐标系 X 轴连续旋转。',
            '绕参考坐标系 Y 轴连续旋转。',
            '绕参考坐标系 Z 轴连续旋转。')
        for i in range(6):
            self.ps_label[i]=wx.StaticText(self.panel, label=cart_display_names[i])
            cart_grid.Add(self.ps_label[i], 0, wx.ALIGN_CENTER_VERTICAL)
            self.pp_button[i]=wx.Button(self.panel, label=cart_names[i]+' +')
            self.pp_button[i].SetToolTip('按住：'+cart_names[i]+' 正方向连续点动；松开：Stop')
            self.pp_button[i].Bind(
                wx.EVT_LEFT_DOWN, lambda evt, mark=i+1: self.teleop_pcs(evt, mark))
            self.pp_button[i].Bind(
                wx.EVT_LEFT_UP, lambda evt, mark=i+1: self.release_button(evt, mark))
            cart_grid.Add(self.pp_button[i], 0, wx.EXPAND)
            self.pm_button[i]=wx.Button(self.panel, label=cart_names[i]+' -')
            self.pm_button[i].SetToolTip('按住：'+cart_names[i]+' 负方向连续点动；松开：Stop')
            self.pm_button[i].Bind(
                wx.EVT_LEFT_DOWN, lambda evt, mark=-1*(i+1): self.teleop_pcs(evt, mark))
            self.pm_button[i].Bind(
                wx.EVT_LEFT_UP, lambda evt, mark=-1*(i+1): self.release_button(evt, mark))
            cart_grid.Add(self.pm_button[i], 0, wx.EXPAND)
            self.ps_display[i]=wx.TextCtrl(
                self.panel, style=wx.TE_CENTER | wx.TE_READONLY,
                value='--', size=(95, -1))
            cart_grid.Add(self.ps_display[i], 0, wx.EXPAND | wx.ALIGN_CENTER_VERTICAL)
            cart_grid.Add(self.make_description(
                self.panel, cart_descriptions[i]+' 按住连续运动，松开即停。', 250),
                0, wx.ALIGN_CENTER_VERTICAL)
        cart_grid.AddGrowableCol(4, 1)
        cart_box.Add(cart_grid, 0, wx.ALL | wx.EXPAND, 10)
        motion_column.Add(cart_box, 0, wx.EXPAND)
        self.main_sizer.Add(
            motion_column, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 12)

        io_box=wx.StaticBoxSizer(wx.StaticBox(self.panel, label='末端 12 芯工具 I/O'), wx.VERTICAL)
        self.io_status_show=wx.StaticText(
            self.panel, label='I/O 状态：等待 read_di/read_do 服务')
        io_box.Add(self.io_status_show, 0, wx.ALL | wx.EXPAND, 10)
        io_grid=wx.FlexGridSizer(rows=7, cols=3, vgap=8, hgap=10)
        for heading in ('实体标签', '状态/操作', '用途与电气注意'):
            label=wx.StaticText(self.panel, label=heading)
            label.SetFont(label.GetFont().Bold())
            io_grid.Add(label, 0, wx.ALIGN_CENTER_VERTICAL)
        for i in range(3):
            io_grid.Add(wx.StaticText(self.panel, label='OUTPUT_'+str(i)+' / DO'+str(i)),
                        0, wx.ALIGN_CENTER_VERTICAL)
            self.DO_btn_display[i]=wx.Button(self.panel, label='DO'+str(i)+'：未知')
            self.DO_btn_display[i].SetToolTip(
                '读取当前输出寄存器，只切换 OUTPUT_'+str(i)+' 对应位，再写后回读')
            self.DO_btn_display[i].Bind(
                wx.EVT_BUTTON,
                lambda evt, marker=i, cl=self.call_write_DO:
                self.call_write_DO_command(evt, marker, cl))
            io_grid.Add(self.DO_btn_display[i], 0, wx.EXPAND)
            io_grid.Add(self.make_description(
                self.panel,
                '点击只切换该路，保留其他位并写后回读。最大 0.4 A；手册的 PNP 与'
                '“导通到 GND”文字互相矛盾，接负载前须台架确认。', 480),
                0, wx.ALIGN_CENTER_VERTICAL)
        for i in range(3):
            io_grid.Add(wx.StaticText(self.panel, label='INPUT_'+str(i)+' / DI'+str(i)),
                        0, wx.ALIGN_CENTER_VERTICAL)
            self.DI_display[i]=wx.TextCtrl(
                self.panel, style=wx.TE_CENTER | wx.TE_READONLY,
                value='DI'+str(i)+'：未知')
            io_grid.Add(self.DI_display[i], 0, wx.EXPAND)
            io_grid.Add(self.make_description(
                self.panel,
                '只读显示，不发送电气命令。PNP 输入：11-30 V 为 ON，浮空由弱下拉保持 OFF。',
                480),
                0, wx.ALIGN_CENTER_VERTICAL)
        io_grid.AddGrowableCol(2, 1)
        io_box.Add(io_grid, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 10)
        io_box.Add(self.make_description(
            self.panel,
            'AI0/AI1、RS-485 A/B 和 24V/GND 是真实物理端子，但当前 ROS 驱动没有可用的'
            '模拟量或 485 协议接口；灯环是机器人状态灯，不是这里的用户 DO。', 820),
            0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 10)
        self.main_sizer.Add(io_box, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 12)

        tool_button_box=wx.StaticBoxSizer(
            wx.StaticBox(self.panel, label='末端实体 POINT / FREE 按键'), wx.VERTICAL)
        self.tool_button_mapping_status=wx.StaticText(
            self.panel,
            label=('本机固定映射：DI 原始字 bit 4 = POINT，bit 5 = FREE。'
                   '它们不是外接端子 INPUT_0..2。'))
        self.tool_button_mapping_status.Wrap(820)
        tool_button_box.Add(
            self.tool_button_mapping_status, 0, wx.ALL | wx.EXPAND, 10)
        tool_grid=wx.FlexGridSizer(rows=8, cols=3, vgap=8, hgap=10)
        for heading in ('实体按键 / 软件入口', '状态', '操作与用途'):
            label=wx.StaticText(self.panel, label=heading)
            label.SetFont(label.GetFont().Bold())
            tool_grid.Add(label, 0, wx.ALIGN_CENTER_VERTICAL)
        self.tool_button_state_show={}
        for name in ('POINT', 'FREE'):
            tool_grid.Add(wx.StaticText(self.panel, label=name), 0, wx.ALIGN_CENTER_VERTICAL)
            state=wx.TextCtrl(
                self.panel, style=wx.TE_CENTER | wx.TE_READONLY,
                value='DI bit '+str(self.tool_button_bits[name])+'：等待 I/O')
            self.tool_button_state_show[name]=state
            tool_grid.Add(state, 0, wx.EXPAND)
            if name == 'POINT':
                behavior='按下沿持久记录姿态'
                description=(
                    '固定读取 DI bit 4。管理节点把六轴弧度和时间写入 '
                    '~/.ros/elfin_freedrive_points.yaml；不会发送轨迹。')
            else:
                behavior='按下立即进入；松开退出'
                description=(
                    '固定读取 DI bit 5。新的按下沿立即请求有界重力补偿；松开后等待静止，'
                    '再恢复当前位置保持。它不会调用任何松抱闸服务。')
            behavior_column=wx.BoxSizer(wx.VERTICAL)
            behavior_show=wx.TextCtrl(
                self.panel, style=wx.TE_CENTER | wx.TE_READONLY, value=behavior)
            behavior_column.Add(behavior_show, 0, wx.EXPAND | wx.BOTTOM, 4)
            behavior_column.Add(
                self.make_description(self.panel, description, 460), 0, wx.EXPAND)
            tool_grid.Add(behavior_column, 0, wx.EXPAND)

        tool_grid.Add(wx.StaticText(self.panel, label='软件进入拖拽'),
                      0, wx.ALIGN_CENTER_VERTICAL)
        software_enter_state=wx.TextCtrl(
            self.panel, style=wx.TE_CENTER | wx.TE_READONLY,
            value='与实体 FREE 共用管理器')
        tool_grid.Add(software_enter_state, 0, wx.EXPAND)
        enter_column=wx.BoxSizer(wx.VERTICAL)
        self.enter_freedrive_btn=wx.Button(self.panel, label='进入零力拖拽')
        self.enter_freedrive_btn.SetToolTip(
            '仅在管理器 READY、六轴静止、Servo On 且无 Fault 时切换控制器')
        enter_column.Add(self.enter_freedrive_btn, 0, wx.EXPAND | wx.BOTTOM, 4)
        enter_column.Add(self.make_description(
            self.panel,
            '软件入口不会绕过门禁。普通硬件启动默认 LOCKED；只有显式 --freedrive 启动才可进入。',
            460), 0, wx.EXPAND)
        tool_grid.Add(enter_column, 0, wx.EXPAND)

        tool_grid.Add(wx.StaticText(self.panel, label='软件退出拖拽'),
                      0, wx.ALIGN_CENTER_VERTICAL)
        software_exit_state=wx.TextCtrl(
            self.panel, style=wx.TE_CENTER | wx.TE_READONLY,
            value='先等待六轴静止')
        tool_grid.Add(software_exit_state, 0, wx.EXPAND)
        exit_column=wx.BoxSizer(wx.VERTICAL)
        self.exit_freedrive_btn=wx.Button(self.panel, label='退出并保持当前位置')
        self.exit_freedrive_btn.SetToolTip(
            '请求退出重力补偿，速度稳定后恢复位置控制器')
        self.exit_freedrive_btn.Disable()
        exit_column.Add(self.exit_freedrive_btn, 0, wx.EXPAND | wx.BOTTOM, 4)
        exit_column.Add(self.make_description(
            self.panel,
            '与松开实体 FREE 完全相同；先增强阻尼减速，再严格确认静止并重试位置控制。必要时先进入驱动器当前位置保持。',
            460), 0, wx.EXPAND)
        tool_grid.Add(exit_column, 0, wx.EXPAND)

        tool_grid.Add(wx.StaticText(self.panel, label='拖拽速度保护'),
                      0, wx.ALIGN_CENTER_VERTICAL)
        self.freedrive_speed_state_show=wx.TextCtrl(
            self.panel,
            style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_WORDWRAP,
            value='当前 100%；等待六轴硬上限')
        self.freedrive_speed_state_show.SetMinSize((-1, 52))
        tool_grid.Add(self.freedrive_speed_state_show, 0, wx.EXPAND)
        speed_column=wx.BoxSizer(wx.VERTICAL)
        speed_row=wx.BoxSizer(wx.HORIZONTAL)
        speed_row.Add(wx.StaticText(self.panel, label='50%'),
                      0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 6)
        self.freedrive_speed_slider=wx.Slider(
            self.panel, value=100, minValue=50, maxValue=200,
            style=wx.SL_HORIZONTAL)
        speed_row.Add(self.freedrive_speed_slider, 1,
                      wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 6)
        speed_row.Add(wx.StaticText(self.panel, label='200%'),
                      0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 8)
        self.freedrive_speed_apply_btn=wx.Button(
            self.panel, label='应用 100%')
        self.freedrive_speed_apply_btn.SetToolTip(
            '仅在零力控制器未运行时应用；下一次进入拖拽时生效')
        speed_row.Add(self.freedrive_speed_apply_btn, 0, wx.ALIGN_CENTER_VERTICAL)
        speed_column.Add(speed_row, 0, wx.EXPAND | wx.BOTTOM, 4)
        speed_column.Add(self.make_description(
            self.panel,
            '只调整零力拖拽的软减速和超速退出阈值。100% 是原值；不改变点动速度、力矩、关节角限位或 Fault 保护。',
            460), 0, wx.EXPAND)
        tool_grid.Add(speed_column, 0, wx.EXPAND)

        tool_grid.Add(wx.StaticText(self.panel, label='六轴拖拽阻尼'),
                      0, wx.ALIGN_CENTER_VERTICAL)
        self.freedrive_damping_state_show=wx.TextCtrl(
            self.panel,
            style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_WORDWRAP,
            value='当前：J1--J6 均为 100%')
        self.freedrive_damping_state_show.SetMinSize((-1, 52))
        tool_grid.Add(self.freedrive_damping_state_show, 0, wx.EXPAND)
        damping_column=wx.BoxSizer(wx.VERTICAL)
        damping_grid=wx.FlexGridSizer(rows=2, cols=6, vgap=4, hgap=6)
        self.freedrive_damping_inputs=[]
        for index in range(6):
            joint_column=wx.BoxSizer(wx.VERTICAL)
            joint_column.Add(wx.StaticText(self.panel, label='J'+str(index+1)),
                             0, wx.ALIGN_CENTER_HORIZONTAL | wx.BOTTOM, 2)
            control=wx.SpinCtrlDouble(
                self.panel, min=0.25, max=2.0, initial=1.0, inc=0.05,
                style=wx.SP_ARROW_KEYS)
            control.SetDigits(2)
            control.SetMinSize((72, -1))
            self.freedrive_damping_inputs.append(control)
            joint_column.Add(control, 0, wx.EXPAND)
            damping_grid.Add(joint_column, 0, wx.EXPAND)
        self.freedrive_damping_apply_btn=wx.Button(
            self.panel, label='应用六轴阻尼')
        self.freedrive_damping_apply_btn.SetToolTip(
            '25% 更轻、200% 更稳；只允许在零力控制器未运行时修改')
        damping_column.Add(damping_grid, 0, wx.EXPAND | wx.BOTTOM, 5)
        damping_column.Add(self.freedrive_damping_apply_btn,
                           0, wx.EXPAND | wx.BOTTOM, 4)
        damping_column.Add(self.make_description(
            self.panel,
            '倍率 1.00=当前默认；降低某轴可减小速度相关阻力。它不修改重力补偿，过低会更容易快速运动，仍受上面的速度保护。',
            460), 0, wx.EXPAND)
        tool_grid.Add(damping_column, 0, wx.EXPAND)

        tool_grid.Add(wx.StaticText(self.panel, label='软件姿态记录'),
                      0, wx.ALIGN_CENTER_VERTICAL)
        self.freedrive_point_count_show=wx.TextCtrl(
            self.panel, style=wx.TE_CENTER | wx.TE_READONLY,
            value='已记录 0 个姿态')
        tool_grid.Add(self.freedrive_point_count_show, 0, wx.EXPAND)
        record_column=wx.BoxSizer(wx.VERTICAL)
        self.record_point_btn=wx.Button(self.panel, label='现在记录当前姿态')
        self.record_point_btn.SetToolTip(
            '调用管理节点持久记录当前六轴姿态，不发送轨迹')
        record_column.Add(self.record_point_btn, 0, wx.EXPAND | wx.BOTTOM, 4)
        record_column.Add(self.make_description(
            self.panel,
            '与实体 POINT 使用同一记录服务。它记录离散标定位姿，不等于录制连续运动轨迹。',
            460), 0, wx.EXPAND)
        tool_grid.Add(record_column, 0, wx.EXPAND)
        tool_grid.AddGrowableCol(1, 1)
        tool_grid.AddGrowableCol(2, 1)
        tool_button_box.Add(tool_grid, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 10)
        self.main_sizer.Add(
            tool_button_box, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 12)

        result_box=wx.StaticBoxSizer(wx.StaticBox(self.panel, label='最近结果 / 错误详情'), wx.VERTICAL)
        self.reply_show=wx.TextCtrl(
            self.panel,
            style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_WORDWRAP,
            value='等待操作。长错误可在这里完整换行并滚动查看。')
        self.reply_show.SetMinSize((-1, 100))
        result_box.Add(self.reply_show, 1, wx.ALL | wx.EXPAND, 10)
        self.main_sizer.Add(result_box, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 12)
    def velocity_setting_cb(self, event):
        current_velocity_scaling=self.velocity_setting.GetValue()*0.01
        try:
            self.teleop_api_dynamic_reconfig_client.update_configuration(
                {'velocity_scaling': current_velocity_scaling})
            self.update_velocity_scaling_show(current_velocity_scaling)
        except Exception as error:
            self.show_local_result(False, '速度倍率更新失败：'+str(error))
        if event is not None:
            event.Skip()
    
    def basic_api_reconfigure_cb(self, config):
        wx.CallAfter(self.apply_velocity_scaling, config.velocity_scaling)

    def apply_velocity_scaling(self, value):
        value=max(0.01, min(1.0, float(value)))
        self.velocity_setting.SetValue(int(round(value*100)))
        self.update_velocity_scaling_show(value)
    
    def action_stop(self):
        self.action_client.wait_for_server(timeout=rospy.Duration(secs=0.5))
        self.action_goal.trajectory.header.stamp.secs=0
        self.action_goal.trajectory.header.stamp.nsecs=0
        self.action_goal.trajectory.points=[]
        self.action_client.send_goal(self.action_goal)
    
    def teleop_joints(self,event,mark):       
        self.call_teleop_joint_req.data=mark
        resp=self.call_teleop_joint.call(self.call_teleop_joint_req)
        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()
        
    def teleop_pcs(self,event,mark): 
        self.call_teleop_cart_req.data=mark            
        resp=self.call_teleop_cart.call(self.call_teleop_cart_req)
        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()    
    
    def release_button(self, event, mark):
        self.call_teleop_stop_req.data=True
        resp=self.call_teleop_stop.call(self.call_teleop_stop_req)
        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()
    
    def call_set_bool_common(self, event, client, request):
        btn=event.GetEventObject()
        check_list=['Servo On', 'Servo Off', 'Clear Fault']
        
        # Check servo state
        if btn.GetName()=='Servo On':
            servo_enabled=bool()
            if self.servo_state_lock.acquire():
                servo_enabled=self.servo_state
                self.servo_state_lock.release()
            if servo_enabled:
                resp=SetBoolResponse()
                resp.success=False
                resp.message='机器人已经处于 Servo On，未重复上电。'
                wx.CallAfter(self.update_reply_show, resp)
                event.Skip()
                return
        
        # Check fault state
        if btn.GetName()=='Clear Fault':
            fault_flag=bool()
            if self.fault_state_lock.acquire():
                fault_flag=self.fault_state
                self.fault_state_lock.release()
            if not fault_flag:
                resp=SetBoolResponse()
                resp.success=False
                resp.message='当前没有 Fault，不需要清故障。'
                wx.CallAfter(self.update_reply_show, resp)
                event.Skip()
                return
        
        # Check if the button is in check list
        if btn.GetName() in check_list:
            self.show_message_dialog(btn.GetName(), client, request)
        else:
            try:
                resp=client.call(request)
                wx.CallAfter(self.update_reply_show, resp)
            except rospy.ServiceException as e:
                resp=SetBoolResponse()
                resp.success=False
                resp.message='当前仿真没有提供该服务。'
                wx.CallAfter(self.update_reply_show, resp)
        event.Skip()
    
    def thread_bg(self, msg, client, request):
        wx.CallAfter(self.show_dialog)
        if msg=='Servo Off':
            self.action_stop()
        rospy.sleep(1)
        try:
            resp=client.call(request)
            wx.CallAfter(self.update_reply_show, resp)
        except rospy.ServiceException as e:
            resp=SetBoolResponse()
            resp.success=False
            resp.message='当前仿真没有提供该服务。'
            wx.CallAfter(self.update_reply_show, resp)
        wx.CallAfter(self.destroy_dialog)

    def process_DO_btn(self, value):
        """Decode the three E05 output bits from the driver's bit-12 map."""
        value=int(value)
        with self.DO_btn_lock:
            for i in range(len(self.DO_btn)):
                self.DO_btn[i]=(value >> (12+i)) & 0x01

    def process_DI_btn(self, value):
        """Keep the raw 16-bit input word; the driver returns it unshifted."""
        raw=int(value) & 0xffff
        with self.DI_show_lock:
            self.di_raw_value=raw
            self.di_seen=True
            for i in range(len(self.DI_show)):
                self.DI_show[i]=(raw >> i) & 0x01
        with self.tool_button_lock:
            for name, bit in self.tool_button_bits.items():
                self.tool_button_pressed[name]=bool((raw >> bit) & 0x01)

    def _note_io_failure(self, error):
        self.io_online=False
        self.io_poll_failures+=1
        self.io_retry_after=time.monotonic()+min(10.0, 2.0**min(self.io_poll_failures, 3))
        message=str(error)
        if message != self.last_io_error:
            self.last_io_error=message
            wx.CallAfter(
                self.update_io_status,
                'I/O 服务暂不可用；Panel 将退避重试（只读）。原因：'+message,
                False)

    def monitor_DO_DI(self, evt):
        """Read I/O at a low rate and back off when the driver is absent."""
        if time.monotonic() < self.io_retry_after:
            return
        try:
            di_value=self.call_read_di.call(self.call_read_di_req).digital_input
            do_value=self.call_read_do.call(self.call_read_do_req).digital_input
        except (rospy.ServiceException, rospy.ROSException) as error:
            self._note_io_failure(error)
            return
        self.io_online=True
        self.io_poll_failures=0
        self.io_retry_after=0.0
        self.last_io_error=''
        self.process_DI_btn(di_value)
        self.process_DO_btn(do_value)
        wx.CallAfter(self.update_io_status, 'I/O 服务在线；输入/输出均按驱动原始位显示。', True)
        wx.CallAfter(self.refresh_io_visuals)

    def update_io_status(self, text, online):
        self.io_status_show.SetLabel(text)
        self.io_status_show.Wrap(1080)
        self.io_status_show.SetForegroundColour(
            wx.Colour(40, 120, 40) if online else wx.Colour(170, 45, 35))
        self.panel.Layout()
        self.panel.FitInside()

    def call_read_DO_command(self):
        """Compatibility helper for callers that request one output read."""
        try:
            value=self.call_read_do.call(self.call_read_do_req).digital_input
            self.process_DO_btn(value)
            return True
        except (rospy.ServiceException, rospy.ROSException) as error:
            self._note_io_failure(error)
            return False

    def call_read_DI_command(self):
        """Compatibility helper for callers that request one input read."""
        try:
            value=self.call_read_di.call(self.call_read_di_req).digital_input
            self.process_DI_btn(value)
            return True
        except (rospy.ServiceException, rospy.ROSException) as error:
            self._note_io_failure(error)
            return False

    def call_write_DO_command(self, event, marker, client):
        if marker < 0 or marker >= len(self.DO_btn_display):
            self.show_local_result(False, '无效的末端数字输出通道：'+str(marker))
            return
        bit = 12 + marker
        output_mask = sum(1 << (12+i) for i in range(len(self.DO_btn_display)))
        label = 'DO'+str(marker)+' / OUTPUT_'+str(marker)
        result = SetBoolResponse()
        try:
            current = self.call_read_do.call(self.call_read_do_req).digital_input
            request = current ^ (1 << bit)
            response = client.call(request)
            if not response.success:
                result.success = False
                result.message = label + ' write was rejected by the driver'
            else:
                observed = self.call_read_do.call(self.call_read_do_req).digital_input
                self.process_DO_btn(observed)
                result.success = ((observed & output_mask) == (request & output_mask))
                if result.success:
                    state = 'ON' if ((observed >> bit) & 0x01) else 'OFF'
                    result.message = label + ' 当前为 '+state+'（写后回读确认）'
                else:
                    result.message = label + ' 写后回读不一致；Panel 未继续写入'
        except (rospy.ServiceException, rospy.ROSException) as e:
            result.success = False
            result.message = label+' 服务不可用：'+str(e)
        wx.CallAfter(self.update_reply_show, result)
        wx.CallAfter(self.refresh_io_visuals)
        event.Skip()

    def refresh_io_visuals(self):
        with self.DO_btn_lock:
            output_states=list(self.DO_btn)
        with self.DI_show_lock:
            input_states=list(self.DI_show[:3])
        for i, state in enumerate(output_states):
            self.DO_btn_display[i].SetLabel(
                'DO'+str(i)+'：'+('ON' if state else 'OFF'))
            self.DO_btn_display[i].SetBackgroundColour(
                wx.Colour(200, 235, 200) if state else wx.NullColour)
        for i, state in enumerate(input_states):
            self.DI_display[i].SetValue(
                'DI'+str(i)+'：'+('ON' if state else 'OFF'))
            self.DI_display[i].SetBackgroundColour(
                wx.Colour(200, 235, 200) if state else wx.NullColour)
        self.refresh_tool_button_visuals()

    def refresh_tool_button_visuals(self):
        with self.DI_show_lock:
            di_seen=self.di_seen
        with self.tool_button_lock:
            bits=dict(self.tool_button_bits)
            pressed=dict(self.tool_button_pressed)
        for name in ('POINT', 'FREE'):
            bit=bits[name]
            if not di_seen:
                self.tool_button_state_show[name].SetValue(
                    'DI bit '+str(bit)+'：等待 I/O')
                self.tool_button_state_show[name].SetBackgroundColour(wx.NullColour)
            else:
                state='按下' if pressed[name] else '释放'
                self.tool_button_state_show[name].SetValue(
                    'DI bit '+str(bit)+'：'+state)
                self.tool_button_state_show[name].SetBackgroundColour(
                    wx.Colour(200, 235, 200) if pressed[name] else wx.NullColour)

    def request_freedrive(self, event, enable):
        self.enter_freedrive_btn.Disable()
        self.exit_freedrive_btn.Disable()
        self.show_local_result(
            True, '正在请求'+('进入零力拖拽...' if enable else '退出并保持当前位置...'))
        worker=threading.Thread(target=self.freedrive_service_worker, args=(enable,))
        worker.daemon=True
        worker.start()
        if event is not None:
            event.Skip()

    def freedrive_service_worker(self, enable):
        try:
            self.call_set_freedrive.wait_for_service(timeout=1.0)
            response=self.call_set_freedrive.call(SetBoolRequest(data=enable))
        except (rospy.ROSException, rospy.ServiceException) as error:
            response=SetBoolResponse(
                success=False,
                message='elfin_freedrive_manager 服务不可用：'+str(error))
        wx.CallAfter(self.finish_freedrive_request, response)

    def finish_freedrive_request(self, response):
        self.update_reply_show(response)
        with self.freedrive_state_lock:
            state=self.freedrive_state
        self.apply_freedrive_state_to_controls(state)

    def preview_freedrive_speed_scale(self, event=None):
        value=self.freedrive_speed_slider.GetValue()
        self.freedrive_speed_apply_btn.SetLabel('应用 '+str(value)+'%')
        if event is not None:
            event.Skip()

    def request_freedrive_speed_scale(self, event=None):
        scale=self.freedrive_speed_slider.GetValue()*0.01
        self.freedrive_speed_apply_btn.Disable()
        self.show_local_result(True, '正在设置拖拽速度保护倍率...')
        worker=threading.Thread(
            target=self.freedrive_speed_scale_worker, args=(scale,))
        worker.daemon=True
        worker.start()
        if event is not None:
            event.Skip()

    def freedrive_speed_scale_worker(self, scale):
        try:
            self.call_set_freedrive_velocity_scale.wait_for_service(timeout=1.0)
            response=self.call_set_freedrive_velocity_scale.call(
                SetFloat64Request(data=scale))
        except (rospy.ROSException, rospy.ServiceException) as error:
            response=SetBoolResponse(
                success=False,
                message='拖拽速度设置服务不可用：'+str(error))
        wx.CallAfter(self.finish_freedrive_speed_scale_request, response)

    def finish_freedrive_speed_scale_request(self, response):
        self.update_reply_show(response)
        with self.freedrive_state_lock:
            state=self.freedrive_state
        self.apply_freedrive_state_to_controls(state)

    def request_freedrive_damping_scales(self, event=None):
        scales=[control.GetValue() for control in self.freedrive_damping_inputs]
        self.freedrive_damping_apply_btn.Disable()
        self.show_local_result(True, '正在设置六轴拖拽阻尼...')
        worker=threading.Thread(
            target=self.freedrive_damping_scales_worker, args=(scales,))
        worker.daemon=True
        worker.start()
        if event is not None:
            event.Skip()

    def freedrive_damping_scales_worker(self, scales):
        try:
            self.call_set_freedrive_damping_scales.wait_for_service(timeout=1.0)
            response=self.call_set_freedrive_damping_scales.call(
                SetDampingScalesRequest(scales=scales))
        except (rospy.ROSException, rospy.ServiceException) as error:
            response=SetBoolResponse(
                success=False,
                message='拖拽阻尼设置服务不可用：'+str(error))
        wx.CallAfter(self.finish_freedrive_damping_scales_request, response)

    def finish_freedrive_damping_scales_request(self, response):
        self.update_reply_show(response)
        with self.freedrive_state_lock:
            state=self.freedrive_state
        self.apply_freedrive_state_to_controls(state)

    def request_record_point(self, event=None):
        self.record_point_btn.Disable()
        worker=threading.Thread(target=self.record_point_worker)
        worker.daemon=True
        worker.start()
        if event is not None:
            event.Skip()

    def record_point_worker(self):
        try:
            self.call_record_freedrive_point.wait_for_service(timeout=1.0)
            response=self.call_record_freedrive_point.call(TriggerRequest())
        except (rospy.ROSException, rospy.ServiceException) as error:
            response=SetBoolResponse(
                success=False,
                message='姿态记录服务不可用：'+str(error))
        wx.CallAfter(self.finish_record_point_request, response)

    def finish_record_point_request(self, response):
        self.record_point_btn.Enable(True)
        self.update_reply_show(response)

    def show_local_result(self, success, message):
        response=SetBoolResponse()
        response.success=bool(success)
        response.message=str(message)
        self.update_reply_show(response)
    
    def show_message_dialog(self, message, cl, rq):
        msg='正在执行：'+message
        self.dlg_label.SetLabel(msg)
        # Let the sizer derive the client height from the rendered text.  A
        # width-only wrap limit prevents a long service name from producing a
        # screen-wide dialog, while no vertical dimension is hard-coded.
        self.dlg_label.Wrap(560)
        self.dlg_sizer.Layout()
        self.dlg_sizer.Fit(self.dlg)
        t=threading.Thread(target=self.thread_bg, args=(message, cl, rq,))
        t.daemon=True
        t.start()
        
    def show_dialog(self):
        self.dlg.SetPosition((self.GetPosition()[0]+250,
                              self.GetPosition()[1]+250))
        self.dlg.ShowModal()
        
    def destroy_dialog(self):
        self.dlg.EndModal(0)
        
    def closewindow(self,event):
        if self.dlg.IsModal():
            self.dlg.EndModal(wx.ID_CANCEL)
        else:
            self.dlg.Hide()

    def init_maintenance_dialog(self):
        self.maintenance_dlg=wx.Dialog(self, title='维护与接口诊断',
                                       style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
        self.maintenance_dlg.Bind(wx.EVT_CLOSE, self.hide_maintenance_dialog)

        root_sizer=wx.BoxSizer(wx.VERTICAL)
        warning=wx.StaticText(
            self.maintenance_dlg,
            label=('重要：松抱闸没有重力补偿。每个服务会同时影响一对关节，单次最多 5 秒，'
                   '随后 Panel 自动请求抱闸。只有额定吊具承重、清空扫掠/夹点区并有人守电闸时才可继续。'))
        warning.Wrap(760)
        warning.SetForegroundColour(wx.Colour(170, 70, 0))
        root_sizer.Add(warning, 0, wx.ALL | wx.EXPAND, 12)

        self.brake_support_confirm=wx.CheckBox(
            self.maintenance_dlg,
            label=('我已确认额定支撑正在承重、扫掠/夹点区无人，并已约束意外转动（仅用于受保护维护）。'))
        root_sizer.Add(self.brake_support_confirm, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 12)

        brake_grid=wx.FlexGridSizer(rows=4, cols=5, vgap=8, hgap=8)
        for heading in ('模块 / 关节', '受保护松闸', '立即抱闸', '模块清故障', '按钮作用与限制'):
            label=wx.StaticText(self.maintenance_dlg, label=heading)
            label.SetFont(label.GetFont().Bold())
            brake_grid.Add(label, 0, wx.ALIGN_CENTER_VERTICAL)

        module_labels=('Slave 1: J2 + J1', 'Slave 2: J3 + J4', 'Slave 3: J5 + J6')
        self.brake_release_buttons=[]
        self.brake_close_buttons=[]
        self.module_reset_buttons=[]
        self.brake_release_events=[None, None, None]
        self.brake_release_threads=[None, None, None]
        self.brake_close_confirmed=[True, True, True]
        self.pending_main_close=False
        self.brake_open_clients=[]
        self.brake_close_clients=[]
        self.module_reset_clients=[]

        for index, module_label in enumerate(module_labels):
            slave=index+1
            brake_grid.Add(wx.StaticText(self.maintenance_dlg, label=module_label),
                           0, wx.ALIGN_CENTER_VERTICAL)

            release_button=wx.Button(self.maintenance_dlg, label='松闸 5 秒')
            release_button.SetToolTip('需要额定支撑确认；最多 5 秒后自动请求抱闸')
            release_button.Bind(wx.EVT_BUTTON,
                                lambda evt, slave_no=slave: self.request_brake_release(evt, slave_no))
            brake_grid.Add(release_button, 0, wx.EXPAND)
            self.brake_release_buttons.append(release_button)

            close_button=wx.Button(self.maintenance_dlg, label='立即抱闸')
            close_button.SetToolTip('立即请求该模块的两个抱闸闭合')
            close_button.Bind(wx.EVT_BUTTON,
                              lambda evt, slave_no=slave: self.request_brake_close(evt, slave_no))
            brake_grid.Add(close_button, 0, wx.EXPAND)
            self.brake_close_buttons.append(close_button)

            reset_button=wx.Button(self.maintenance_dlg, label='模块清故障')
            reset_button.SetToolTip('仅 Servo Off 时请求该模块清故障，不会移动机器人')
            reset_button.Bind(wx.EVT_BUTTON,
                              lambda evt, slave_no=slave: self.request_module_fault_reset(evt, slave_no))
            brake_grid.Add(reset_button, 0, wx.EXPAND)
            self.module_reset_buttons.append(reset_button)
            brake_grid.Add(self.make_description(
                self.maintenance_dlg,
                '松闸会同时释放 '+module_label.split(':')[1].strip()+
                '；清故障前先排除机械接触/供电原因。', 360),
                0, wx.ALIGN_CENTER_VERTICAL)

            self.brake_open_clients.append(
                rospy.ServiceProxy('/elfin_module_open_brake_slave'+str(slave), SetBool))
            self.brake_close_clients.append(
                rospy.ServiceProxy('/elfin_module_close_brake_slave'+str(slave), SetBool))
            self.module_reset_clients.append(
                rospy.ServiceProxy('/elfin_module_reset_fault_slave'+str(slave), SetBool))

        for column in (1, 2, 3):
            brake_grid.AddGrowableCol(column, 1)
        root_sizer.Add(brake_grid, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 12)

        blocked_label=wx.StaticText(
            self.maintenance_dlg,
            label=('以下接口故意不做成人工按钮：单模块直接使能、自动位置识别、无约束关节命令和力矩主题。'
                   '它们会绕过高层一致性检查，或可能重新标定/驱动机器人。'))
        blocked_label.Wrap(760)
        root_sizer.Add(blocked_label, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 12)

        diagnostic_row=wx.FlexGridSizer(rows=2, cols=2, vgap=8, hgap=10)
        self.refresh_diagnostics_btn=wx.Button(self.maintenance_dlg, label='刷新接口与状态')
        self.refresh_diagnostics_btn.SetToolTip('只读列出 ROS 服务、主题、驱动状态和 PDO 诊断')
        self.refresh_diagnostics_btn.Bind(wx.EVT_BUTTON, self.refresh_maintenance_diagnostics)
        diagnostic_row.Add(self.refresh_diagnostics_btn, 0, wx.EXPAND)
        diagnostic_row.Add(self.make_description(
            self.maintenance_dlg, '只读列出 ROS 服务、主题、驱动状态与 PDO，不发送运动命令。', 700),
            0, wx.ALIGN_CENTER_VERTICAL)
        close_dialog_button=wx.Button(self.maintenance_dlg, wx.ID_CLOSE, label='隐藏窗口')
        close_dialog_button.SetToolTip('关闭维护窗口；不会自动关闭机器人')
        close_dialog_button.Bind(wx.EVT_BUTTON, self.hide_maintenance_dialog)
        diagnostic_row.Add(close_dialog_button, 0, wx.EXPAND)
        diagnostic_row.Add(self.make_description(
            self.maintenance_dlg, '只隐藏此窗口，不改变 Servo、Fault 或机械臂状态。', 700),
            0, wx.ALIGN_CENTER_VERTICAL)
        diagnostic_row.AddGrowableCol(1, 1)
        root_sizer.Add(diagnostic_row, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 12)

        self.maintenance_output=wx.TextCtrl(
            self.maintenance_dlg,
            style=wx.TE_MULTILINE | wx.TE_READONLY | wx.TE_WORDWRAP,
            value='点击“刷新接口与状态”读取在线 ROS 接口和驱动状态。长内容可滚动查看。')
        self.maintenance_output.SetMinSize((-1, 220))
        root_sizer.Add(self.maintenance_output, 1, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 12)

        self.maintenance_dlg.SetSizer(root_sizer)
        display_size=wx.GetDisplaySize()
        display_width=display_size.GetWidth()
        display_height=display_size.GetHeight()
        dialog_width=max(900, min(1120, display_width-100))
        dialog_height=max(560, min(720, display_height-100))
        self.maintenance_dlg.SetMinSize(
            (min(900, dialog_width), min(560, dialog_height)))
        self.maintenance_dlg.SetSize((dialog_width, dialog_height))

    def show_maintenance_dialog(self, event):
        self.maintenance_dlg.CentreOnParent()
        self.maintenance_dlg.Show()
        self.maintenance_dlg.Raise()
        self.refresh_maintenance_diagnostics(None)
        if event is not None:
            event.Skip()

    def hide_maintenance_dialog(self, event):
        for release_event in self.brake_release_events:
            if release_event is not None:
                release_event.set()
        self.maintenance_dlg.Hide()
        if event is not None:
            event.Skip(False)

    def on_main_close(self, event):
        with self.freedrive_state_lock:
            freedrive_state=self.freedrive_state
        if (freedrive_state in (
                'ENTERING', 'ACTIVE', 'EXITING', 'RECOVERING', 'HOLDING', 'FALLBACK')
                and event.CanVeto()):
            self.show_local_result(
                False,
                'Panel 暂停退出：零力拖拽仍在切换或运行。先松开实体 FREE，或点击“退出并保持当前位置”。')
            event.Veto()
            return

        active_releases=[]
        for index, release_event in enumerate(self.brake_release_events):
            active_thread=self.brake_release_threads[index]
            if (release_event is not None and active_thread is not None
                    and active_thread.is_alive()):
                release_event.set()
                active_releases.append(index+1)

        # Do not let the GUI process disappear before an active release worker
        # has sent its close request.  The worker calls finish_pending_main_close
        # after it completes, which generates a second (now safe) close event.
        if active_releases and event.CanVeto():
            self.pending_main_close=True
            self.maintenance_dlg.Show()
            self.maintenance_dlg.Raise()
            self.maintenance_message(
                'Panel 暂停退出：正在请求这些模块抱闸：'
                + ', '.join(str(number) for number in active_releases) + '。', False)
            event.Veto()
            return
        event.Skip()

    def finish_pending_main_close(self):
        if not self.pending_main_close:
            return
        for active_thread in self.brake_release_threads:
            if active_thread is not None and active_thread.is_alive():
                return
        unconfirmed=[str(index+1) for index, confirmed in enumerate(self.brake_close_confirmed)
                     if not confirmed]
        if unconfirmed:
            self.pending_main_close=False
            self.maintenance_message(
                'Panel 保持打开：以下模块未确认抱闸：'
                + ', '.join(unconfirmed) + '。请重试“立即抱闸”或使用上游电闸。',
                False)
            return
        self.pending_main_close=False
        self.Close()

    def maintenance_message(self, message, success=None):
        self.maintenance_output.SetValue(message)
        if success is True:
            self.maintenance_output.SetBackgroundColour(wx.Colour(220, 245, 220))
        elif success is False:
            self.maintenance_output.SetBackgroundColour(wx.Colour(250, 220, 220))
        else:
            self.maintenance_output.SetBackgroundColour(wx.NullColour)

    def get_cached_robot_state(self):
        with self.servo_state_lock:
            servo_enabled=self.servo_state
            servo_state_received=self.servo_state_received
        with self.fault_state_lock:
            faulted=self.fault_state
            fault_state_received=self.fault_state_received
        return servo_enabled, faulted, servo_state_received, fault_state_received

    def set_release_buttons_enabled(self, enabled):
        for button in self.brake_release_buttons:
            button.Enable(enabled)

    def request_brake_release(self, event, slave_no):
        index=slave_no-1
        for other_index, active_thread in enumerate(self.brake_release_threads):
            if active_thread is not None and active_thread.is_alive():
                self.maintenance_message(
                    '拒绝松闸：模块 '+str(other_index+1)+' 已在执行松闸/自动抱闸周期。',
                    False)
                return

        servo_enabled, faulted, servo_received, fault_received=self.get_cached_robot_state()
        if not servo_received or not fault_received:
            self.maintenance_message(
                '拒绝松闸：尚未同时收到最新 Servo 与 Fault 状态。', False)
            return
        if servo_enabled:
            self.maintenance_message('拒绝松闸：当前仍是 Servo On。', False)
            return
        if faulted:
            self.maintenance_message('拒绝松闸：请先诊断并处理驱动 Fault。', False)
            return
        if not self.brake_support_confirm.GetValue():
            self.maintenance_message('拒绝松闸：尚未勾选额定支撑与清场确认。', False)
            return

        motion_client=rospy.ServiceProxy('/elfin_ros_control/elfin/get_motion_state', SetBool)
        try:
            motion_client.wait_for_service(timeout=1.0)
            motion_response=motion_client.call(SetBoolRequest(data=True))
        except (rospy.ROSException, rospy.ServiceException) as error:
            self.maintenance_message('拒绝松闸：无法读取编码器运动状态：'+str(error), False)
            return
        if motion_response.success:
            self.maintenance_message('拒绝松闸：编码器仍检测到运动。', False)
            return

        joint_pair=('J2 与 J1', 'J3 与 J4', 'J5 与 J6')[index]
        confirmation=wx.MessageDialog(
            self.maintenance_dlg,
            ('模块 '+str(slave_no)+' 会同时释放 '+joint_pair+'，且没有重力补偿。\n\n'
             'Panel 最迟 5 秒后请求自动抱闸。仅当额定支撑正在承重、所有人都在扫掠/夹点区外时继续。'),
            '确认受保护松闸',
            wx.YES_NO | wx.NO_DEFAULT | wx.ICON_WARNING)
        accepted=(confirmation.ShowModal()==wx.ID_YES)
        confirmation.Destroy()
        if not accepted:
            self.maintenance_message('操作者已取消松闸。', None)
            return

        release_event=threading.Event()
        self.brake_release_events[index]=release_event
        worker=threading.Thread(target=self.brake_release_worker,
                                args=(slave_no, release_event))
        worker.daemon=True
        self.brake_release_threads[index]=worker
        self.set_release_buttons_enabled(False)
        self.maintenance_message('正在请求模块 '+str(slave_no)+' 受保护松闸...', None)
        worker.start()

    def brake_release_worker(self, slave_no, release_event):
        index=slave_no-1
        request=SetBoolRequest(data=True)
        opened=False
        try:
            self.brake_open_clients[index].wait_for_service(timeout=1.0)
            response=self.brake_open_clients[index].call(request)
            opened=response.success
            if opened:
                self.brake_close_confirmed[index]=False
            wx.CallAfter(self.update_reply_show, response)
            if opened:
                wx.CallAfter(self.maintenance_message,
                             '模块 '+str(slave_no)+' 抱闸已释放；最迟 5 秒后自动请求抱闸。',
                             None)
            else:
                wx.CallAfter(self.maintenance_message,
                             '模块 '+str(slave_no)+' 松闸被驱动拒绝：'+response.message,
                             False)
        except (rospy.ROSException, rospy.ServiceException) as error:
            wx.CallAfter(self.maintenance_message,
                         '模块 '+str(slave_no)+' 松闸服务失败：'+str(error), False)

        if opened:
            release_event.wait(5.0)
            try:
                self.brake_close_clients[index].wait_for_service(timeout=1.0)
                close_response=self.brake_close_clients[index].call(request)
                self.brake_close_confirmed[index]=close_response.success
                wx.CallAfter(self.update_reply_show, close_response)
                wx.CallAfter(self.maintenance_message,
                             ('模块 '+str(slave_no)+' 自动抱闸结果：'+close_response.message),
                             close_response.success)
            except (rospy.ROSException, rospy.ServiceException) as error:
                wx.CallAfter(self.maintenance_message,
                             ('紧急：模块 '+str(slave_no)+' 未确认抱闸：'+str(error)+
                              '。请使用上游电闸。'), False)

        wx.CallAfter(self.set_release_buttons_enabled, True)
        wx.CallAfter(self.brake_support_confirm.SetValue, False)
        self.brake_release_events[index]=None
        self.brake_release_threads[index]=None
        wx.CallAfter(self.finish_pending_main_close)

    def request_brake_close(self, event, slave_no):
        index=slave_no-1
        release_event=self.brake_release_events[index]
        active_thread=self.brake_release_threads[index]
        if release_event is not None and active_thread is not None and active_thread.is_alive():
            release_event.set()
            self.maintenance_message('已请求模块 '+str(slave_no)+' 立即抱闸。', None)
            return

        worker=threading.Thread(target=self.brake_close_worker, args=(slave_no,))
        worker.daemon=True
        worker.start()

    def brake_close_worker(self, slave_no):
        index=slave_no-1
        request=SetBoolRequest(data=True)
        try:
            self.brake_close_clients[index].wait_for_service(timeout=1.0)
            response=self.brake_close_clients[index].call(request)
            if response.success:
                self.brake_close_confirmed[index]=True
            wx.CallAfter(self.update_reply_show, response)
            wx.CallAfter(self.maintenance_message,
                         '模块 '+str(slave_no)+' 抱闸请求：'+response.message,
                         response.success)
        except (rospy.ROSException, rospy.ServiceException) as error:
            wx.CallAfter(self.maintenance_message,
                         ('紧急：模块 '+str(slave_no)+' 未确认抱闸：'+str(error)+
                          '。请使用上游电闸。'), False)
        wx.CallAfter(self.finish_pending_main_close)

    def request_module_fault_reset(self, event, slave_no):
        servo_enabled, unused_faulted, servo_received, fault_received=self.get_cached_robot_state()
        if not servo_received or not fault_received:
            self.maintenance_message('拒绝模块清故障：实时 Servo/Fault 状态未知。', False)
            return
        if servo_enabled:
            self.maintenance_message('拒绝模块清故障：当前仍是 Servo On。', False)
            return
        worker=threading.Thread(target=self.module_fault_reset_worker, args=(slave_no,))
        worker.daemon=True
        worker.start()

    def module_fault_reset_worker(self, slave_no):
        index=slave_no-1
        request=SetBoolRequest(data=True)
        try:
            self.module_reset_clients[index].wait_for_service(timeout=1.0)
            response=self.module_reset_clients[index].call(request)
            wx.CallAfter(self.update_reply_show, response)
            wx.CallAfter(self.maintenance_message,
                         '模块 '+str(slave_no)+' 清故障结果：'+response.message,
                         response.success)
        except (rospy.ROSException, rospy.ServiceException) as error:
            wx.CallAfter(self.maintenance_message,
                         '模块 '+str(slave_no)+' 清故障服务失败：'+str(error), False)

    def refresh_maintenance_diagnostics(self, event):
        self.refresh_diagnostics_btn.Disable()
        self.maintenance_message('正在只读获取 Elfin 在线接口与状态...', None)
        worker=threading.Thread(target=self.maintenance_diagnostics_worker)
        worker.daemon=True
        worker.start()
        if event is not None:
            event.Skip()

    def maintenance_diagnostics_worker(self):
        lines=[]
        servo_enabled, faulted, servo_received, fault_received=self.get_cached_robot_state()
        lines.append('缓存状态：Servo='+str(servo_enabled)+'（已收到='+str(servo_received)+
                     '），Fault='+str(faulted)+'（已收到='+str(fault_received)+'）')
        with self.DI_show_lock:
            di_summary='0x%04x' % self.di_raw_value if self.di_seen else '尚未收到'
        lines.append('末端数字输入原始值：'+di_summary)
        lines.append(
            '实体按键固定映射：POINT=DI bit '+str(self.POINT_DI_BIT)+
            '，FREE=DI bit '+str(self.FREE_DI_BIT))
        with self.freedrive_state_lock:
            freedrive_state=self.freedrive_state
            freedrive_detail=self.freedrive_detail
            freedrive_ring=self.freedrive_ring_state
            point_count=self.freedrive_point_count
            validation=self.freedrive_validation
            trial_log=self.freedrive_trial_log
        lines.append(
            '零力管理器：state='+freedrive_state+'，detail='+freedrive_detail)
        lines.append(
            'FREE 行为：新的按下沿立即请求有界重力补偿，松开后恢复当前位置保持；'
            '任何时候都不调用松抱闸服务。')
        lines.append(
            'POINT 已持久记录 '+str(point_count)+' 个姿态；灯环状态语义='+freedrive_ring+
            '（当前不是已验证的主动灯色回读）。')
        lines.append('重力模型预检：'+validation)
        lines.append('最近拖拽遥测：'+trial_log)

        try:
            service_prefixes=('/elfin', '/controller_manager/', '/move_group/')
            moveit_services=(
                '/apply_planning_scene', '/check_state_validity', '/clear_octomap',
                '/compute_cartesian_path', '/compute_fk', '/compute_ik',
                '/get_planning_scene', '/plan_kinematic_path')
            services=sorted(
                name for name in rosservice.get_service_list()
                if name.startswith(service_prefixes) or name in moveit_services)
        except Exception as error:
            services=[]
            lines.append('服务清单读取失败：'+str(error))
        lines.append('\n在线机器人 / MoveIt 服务（'+str(len(services))+'）：\n  '+'\n  '.join(services))

        try:
            topic_prefixes=(
                '/elfin', '/joint_states', '/move_group', '/planning_scene',
                '/collision_object', '/attached_collision_object', '/tf')
            topics=sorted(
                name+'  ['+topic_type+']' for name, topic_type in rospy.get_published_topics()
                if name.startswith(topic_prefixes))
        except Exception as error:
            topics=[]
            lines.append('主题清单读取失败：'+str(error))
        lines.append('\n在线机器人 / MoveIt 主题（'+str(len(topics))+'）：\n  '+'\n  '.join(topics))

        lines.append(
            '\n源码审计：AI0/AI1 与 Smart_Camera_X/Y 虽出现在 slave4 源码名称中，'
            '当前驱动没有解码或发布；slave4 原始 PDO 服务也是全零占位。'
            '这些物理端子目前不是可用的 ROS 数据接口。RS-485 同样没有协议节点。')

        diagnostic_services=(
            '/elfin_ros_control/elfin/get_motion_state',
            '/elfin_ros_control/elfin/get_pos_align_state',
            '/elfin_ros_control/elfin/get_current_position',
            '/elfin_ros_control/elfin/get_txpdo',
            '/elfin_ros_control/elfin/get_rxpdo',
            '/elfin_ros_control/elfin/io_port1/get_txpdo',
            '/elfin_ros_control/elfin/io_port1/get_rxpdo')
        request=SetBoolRequest(data=True)
        for service_name in diagnostic_services:
            if service_name not in services:
                lines.append('\n'+service_name+'：不可用')
                continue
            try:
                client=rospy.ServiceProxy(service_name, SetBool)
                response=client.call(request)
                lines.append('\n'+service_name+'：success='+str(response.success)+'\n'+response.message)
            except rospy.ServiceException as error:
                lines.append('\n'+service_name+'：调用失败：'+str(error))

        wx.CallAfter(self.maintenance_message, '\n'.join(lines), None)
        wx.CallAfter(self.refresh_diagnostics_btn.Enable, True)
    
    def show_set_links_dialog(self, evt):
        self.sld_ref_link_show.SetValue(self.ref_link_name)
        self.sld_end_link_show.SetValue(self.end_link_name)
        self.set_links_dlg.SetPosition((self.GetPosition()[0]+150,
                                        self.GetPosition()[1]+250))
        self.set_links_dlg.ShowModal()
    
    def update_ref_link(self, evt):
        request=SetStringRequest()
        request.data=self.sld_ref_link_show.GetValue()
        
        resp=self.call_set_ref_link.call(request)
        wx.CallAfter(self.update_reply_show, resp)
    
    def update_end_link(self, evt):
        request=SetStringRequest()
        request.data=self.sld_end_link_show.GetValue()
        
        resp=self.call_set_end_link.call(request)
        wx.CallAfter(self.update_reply_show, resp)
    
    def updateDisplay(self, msg):
        if len(msg) < 12:
            return
        for i in range(len(self.js_display)):
            self.js_display[i].SetValue(msg[i])

        for i in range(len(self.ps_display)):
            self.ps_display[i].SetValue(msg[i+6])
            
        if self.ref_link_lock.acquire():
            ref_link=self.ref_link_name
            self.ref_link_lock.release()
        
        if self.end_link_lock.acquire():
            end_link=self.end_link_name
            self.end_link_lock.release()
        
        self.ref_link_show.SetValue(ref_link)
        self.end_link_show.SetValue(end_link)
    
    def update_reply_show(self,msg):
        if msg.success:
            self.reply_show.SetBackgroundColour(wx.Colour(200, 225, 200))
        else:
            self.reply_show.SetBackgroundColour(wx.Colour(225, 200, 200))
        self.reply_show.SetValue(msg.message)
            
    def update_servo_state(self, msg):
        if msg.data:
            self.servo_state_show.SetBackgroundColour(wx.Colour(200, 225, 200))
            self.servo_state_show.SetValue('已使能（Servo On）')
        else:
            self.servo_state_show.SetBackgroundColour(wx.Colour(225, 200, 200))
            self.servo_state_show.SetValue('已关闭（Servo Off）')
    
    def update_fault_state(self, msg):
        if msg.data:
            self.fault_state_show.SetBackgroundColour(wx.Colour(225, 200, 200))
            self.fault_state_show.SetValue('Fault：底层保护触发')
        else:
            self.fault_state_show.SetBackgroundColour(wx.Colour(200, 225, 200))
            self.fault_state_show.SetValue('无 Fault')

    def update_freedrive_state(self, state):
        labels={
            'STARTING': '启动中',
            'WAITING': '等待状态',
            'LOCKED': '真机入口锁定',
            'SERVO_OFF': 'Servo Off',
            'FAULT': 'Fault 锁存',
            'READY': '可进入（READY）',
            'ENTERING': '正在切入重力补偿',
            'ACTIVE': '零力拖拽 ACTIVE',
            'EXITING': '正在静止并恢复位置保持',
            'RECOVERING': '保护减速并重试位置保持',
            'HOLDING': '驱动器当前位置保持',
            'FALLBACK': '正在保护回退',
            'ERROR': '保护回退失败'}
        self.freedrive_state_show.SetValue(labels.get(state, state))
        if state == 'ACTIVE':
            colour=wx.Colour(190, 220, 250)
        elif state == 'READY':
            colour=wx.Colour(200, 235, 200)
        elif state in ('ERROR', 'FAULT', 'FALLBACK'):
            colour=wx.Colour(245, 205, 205)
        elif state in ('RECOVERING', 'HOLDING', 'EXITING'):
            colour=wx.Colour(245, 235, 180)
        elif state == 'LOCKED':
            colour=wx.Colour(235, 225, 190)
        else:
            colour=wx.NullColour
        self.freedrive_state_show.SetBackgroundColour(colour)
        self.apply_freedrive_state_to_controls(state)

    def apply_freedrive_state_to_controls(self, state):
        owns_joints=state in (
            'ENTERING', 'ACTIVE', 'EXITING', 'RECOVERING', 'HOLDING', 'FALLBACK')
        for button in self.jp_button+self.jm_button+self.pp_button+self.pm_button:
            button.Enable(not owns_joints)
        self.home_btn.Enable(not owns_joints)
        self.power_on_btn.Enable(not owns_joints)
        self.velocity_setting.Enable(not owns_joints)
        self.freedrive_speed_slider.Enable(not owns_joints)
        self.freedrive_speed_apply_btn.Enable(not owns_joints)
        for control in self.freedrive_damping_inputs:
            control.Enable(not owns_joints)
        self.freedrive_damping_apply_btn.Enable(not owns_joints)
        self.set_links_btn.Enable(not owns_joints)
        self.enter_freedrive_btn.Enable(not owns_joints)
        self.exit_freedrive_btn.Enable(state in ('ACTIVE', 'EXITING', 'RECOVERING'))
        # These remain available as independent stop paths during a bad switch.
        self.power_off_btn.Enable(True)
        self.stop_btn.Enable(True)

    def update_freedrive_detail(self, detail):
        self.freedrive_detail_show.SetValue(detail)

    def update_freedrive_ring(self, state):
        mapping={
            'RED_FAULT_EXPECTED': ('红色（预期：Fault）', wx.Colour(245, 205, 205)),
            'BLUE_ZERO_FORCE_EXPECTED': ('蓝色（预期：零力示教）', wx.Colour(190, 220, 250)),
            'YELLOW_SERVO_OFF_EXPECTED': ('黄色（预期：Servo Off）', wx.Colour(245, 235, 180)),
            'GREEN_SERVO_ON_EXPECTED': ('绿色（预期：Servo On）', wx.Colour(200, 235, 200)),
            'UNKNOWN_RING_STATE': ('未知（尚无足够状态）', wx.NullColour)}
        label, colour=mapping.get(state, (state, wx.NullColour))
        self.freedrive_ring_show.SetValue(label)
        self.freedrive_ring_show.SetBackgroundColour(colour)

    def update_freedrive_point_count(self, count):
        self.freedrive_point_count_show.SetValue('已记录 '+str(count)+' 个姿态')

    def update_freedrive_validation(self, detail):
        self.freedrive_validation_show.SetValue(detail)
        if detail.startswith('通过'):
            colour=wx.Colour(200, 235, 200)
        elif detail.startswith('警告'):
            colour=wx.Colour(245, 235, 180)
        elif detail.startswith('未通过'):
            colour=wx.Colour(245, 205, 205)
        else:
            colour=wx.NullColour
        self.freedrive_validation_show.SetBackgroundColour(colour)

    def update_freedrive_trial_log(self, path):
        self.freedrive_trial_log_show.SetValue(path or '尚未开始本次试验')

    def refresh_freedrive_speed_limit_show(self):
        with self.freedrive_state_lock:
            scale=self.freedrive_velocity_scale
            hard_limits=list(self.freedrive_velocity_hard_limits)
        text='当前 '+str(round(scale*100, 1))+'%'
        if len(hard_limits) == 6:
            text+='；硬上限：'+'，'.join(
                'J'+str(index+1)+' '+str(round(value*180/math.pi, 1))+' deg/s'
                for index, value in enumerate(hard_limits))
        else:
            text+='；等待六轴硬上限'
        self.freedrive_speed_state_show.SetValue(text)

    def freedrive_velocity_scale_cb(self, data):
        with self.freedrive_state_lock:
            self.freedrive_velocity_scale=float(data.data)
        value=max(50, min(200, int(round(float(data.data)*100))))
        wx.CallAfter(self.freedrive_speed_slider.SetValue, value)
        wx.CallAfter(self.freedrive_speed_apply_btn.SetLabel,
                     '应用 '+str(value)+'%')
        wx.CallAfter(self.refresh_freedrive_speed_limit_show)

    def freedrive_velocity_hard_limits_cb(self, data):
        with self.freedrive_state_lock:
            self.freedrive_velocity_hard_limits=list(data.data)
        wx.CallAfter(self.refresh_freedrive_speed_limit_show)

    def update_freedrive_damping_scales(self, scales):
        if len(scales) != 6:
            return
        for control, value in zip(self.freedrive_damping_inputs, scales):
            control.SetValue(float(value))
        text='当前：'+'，'.join(
            'J'+str(index+1)+' '+str(round(value*100, 1))+'%'
            for index, value in enumerate(scales))
        self.freedrive_damping_state_show.SetValue(text)

    def freedrive_damping_scales_cb(self, data):
        scales=list(data.data)
        with self.freedrive_state_lock:
            self.freedrive_damping_scales=scales
        wx.CallAfter(self.update_freedrive_damping_scales, scales)

    def freedrive_state_cb(self, data):
        with self.freedrive_state_lock:
            self.freedrive_state=data.data
        wx.CallAfter(self.update_freedrive_state, data.data)

    def freedrive_detail_cb(self, data):
        with self.freedrive_state_lock:
            self.freedrive_detail=data.data
        wx.CallAfter(self.update_freedrive_detail, data.data)

    def freedrive_active_cb(self, data):
        with self.freedrive_state_lock:
            self.freedrive_active=bool(data.data)

    def freedrive_ring_cb(self, data):
        with self.freedrive_state_lock:
            self.freedrive_ring_state=data.data
        wx.CallAfter(self.update_freedrive_ring, data.data)

    def freedrive_point_count_cb(self, data):
        with self.freedrive_state_lock:
            self.freedrive_point_count=int(data.data)
        wx.CallAfter(self.update_freedrive_point_count, data.data)

    def freedrive_recorded_point_cb(self, data):
        if len(data.position) < 6:
            return
        pose='；'.join(
            'J'+str(index+1)+'='+str(round(data.position[index]*180/math.pi, 3))+' deg'
            for index in range(6))
        wx.CallAfter(
            self.show_local_result, True,
            'POINT 姿态已持久记录：'+pose)

    def freedrive_validation_cb(self, data):
        with self.freedrive_state_lock:
            self.freedrive_validation=data.data
        wx.CallAfter(self.update_freedrive_validation, data.data)

    def freedrive_trial_log_cb(self, data):
        with self.freedrive_state_lock:
            self.freedrive_trial_log=data.data
        wx.CallAfter(self.update_freedrive_trial_log, data.data)
        
    def update_velocity_scaling_show(self, msg):
        self.velocity_setting_show.SetValue(str(round(float(msg)*100, 1))+'%')
    
    
    def js_call_back(self, data):
        try:
            self.listener.waitForTransform(
                self.group.get_planning_frame(), self.group.get_end_effector_link(),
                rospy.Time(0), rospy.Duration(0.15))
            xyz, qua=self.listener.lookupTransform(
                self.group.get_planning_frame(), self.group.get_end_effector_link(), rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        rpy=tf.transformations.euler_from_quaternion(qua)
        values=[str(round(value*180/math.pi, 2)) for value in data.position]
        values.extend(str(round(value*1000, 2)) for value in xyz)
        values.extend(str(round(value*180/math.pi, 2)) for value in rpy)
        wx.CallAfter(self.updateDisplay, values)
    
    def monitor_status(self, evt):
        try:
            current_joint_values=self.group.get_current_joint_values()
            with self.ref_link_lock:
                ref_link=self.ref_link_name
            with self.end_link_lock:
                end_link=self.end_link_name
            self.listener.waitForTransform(
                ref_link, end_link, rospy.Time(0), rospy.Duration(0.15))
            xyz, qua=self.listener.lookupTransform(ref_link, end_link, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,
                RuntimeError, rospy.ROSException):
            return
        rpy=tf.transformations.euler_from_quaternion(qua)
        values=[str(round(value*180/math.pi, 2)) for value in current_joint_values[:6]]
        values.extend(str(round(value*1000, 2)) for value in xyz)
        values.extend(str(round(value*180/math.pi, 2)) for value in rpy)
        wx.CallAfter(self.updateDisplay, values)
            
    def servo_state_cb(self, data):
        if self.servo_state_lock.acquire():
            self.servo_state=data.data
            self.servo_state_received=True
            self.servo_state_lock.release()
        wx.CallAfter(self.update_servo_state, data)
    
    def fault_state_cb(self, data):
        if self.fault_state_lock.acquire():
            self.fault_state=data.data
            self.fault_state_received=True
            self.fault_state_lock.release()
        wx.CallAfter(self.update_fault_state, data)
    
    def ref_link_name_cb(self, data):
        if self.ref_link_lock.acquire():
            self.ref_link_name=data.data
            self.ref_link_lock.release()
    
    def end_link_name_cb(self, data):
        if self.end_link_lock.acquire():
            self.end_link_name=data.data
            self.end_link_lock.release()
        
    def listen(self):
        rospy.Subscriber(self.elfin_driver_ns+'enable_state', Bool, self.servo_state_cb)
        rospy.Subscriber(self.elfin_driver_ns+'fault_state', Bool, self.fault_state_cb)
        rospy.Subscriber(self.elfin_basic_api_ns+'reference_link_name', String, self.ref_link_name_cb)
        rospy.Subscriber(self.elfin_basic_api_ns+'end_link_name', String, self.end_link_name_cb)
        rospy.Subscriber('/elfin_freedrive_manager/state', String, self.freedrive_state_cb)
        rospy.Subscriber('/elfin_freedrive_manager/state_detail', String, self.freedrive_detail_cb)
        rospy.Subscriber('/elfin_freedrive_manager/active', Bool, self.freedrive_active_cb)
        rospy.Subscriber('/elfin_freedrive_manager/ring_state', String, self.freedrive_ring_cb)
        rospy.Subscriber('/elfin_freedrive_manager/point_count', UInt32,
                         self.freedrive_point_count_cb)
        rospy.Subscriber('/elfin_freedrive_manager/recorded_point', JointState,
                         self.freedrive_recorded_point_cb)
        rospy.Subscriber('/elfin_freedrive_manager/model_validation', String,
                         self.freedrive_validation_cb)
        rospy.Subscriber('/elfin_freedrive_manager/trial_log_path', String,
                         self.freedrive_trial_log_cb)
        rospy.Subscriber('/elfin_freedrive_manager/velocity_limit_scale', Float64,
                         self.freedrive_velocity_scale_cb)
        rospy.Subscriber('/elfin_freedrive_manager/velocity_hard_limits',
                         Float64MultiArray,
                         self.freedrive_velocity_hard_limits_cb)
        rospy.Subscriber('/elfin_freedrive_manager/damping_scales',
                         Float64MultiArray,
                         self.freedrive_damping_scales_cb)

        # Two low-rate timers avoid flooding the ROS master and GUI when the
        # optional end-I/O service is not running.  Status lookup has a bounded
        # TF wait, so a missing transform cannot freeze the Panel.
        rospy.Timer(rospy.Duration(0.5), self.monitor_DO_DI)
        rospy.Timer(rospy.Duration(0.25), self.monitor_status)
  
if __name__=='__main__':  
    rospy.init_node('elfin_gui')
    app=wx.App(False)  
    myframe=MyFrame(parent=None,id=-1)  
    myframe.Show(True)

    myframe.listen()

    app.MainLoop()
