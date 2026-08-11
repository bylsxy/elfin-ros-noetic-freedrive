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
import csv
import glob
import rospy
import math
import os
import re
import signal
import subprocess
import time
from collections import deque
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
    DeleteRecordedPoint, DeleteRecordedPointRequest,
    ListRecordedPoints, ListRecordedPointsRequest,
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
    DRIVER_STATE_TIMEOUT=0.75
    PAYLOAD_CALIBRATION_MINIMUM_FLANGE_HEIGHT=0.45
    PAYLOAD_CALIBRATION_POSE_NAMES=frozenset((
        '拟合 A 低负载中性腕',
        '拟合 B 中负载正俯仰',
        '拟合 C 高负载正腕姿',
        '拟合 D 高负载负腕姿',
        '拟合 E 中负载负俯仰',
        '留出 F 交叉腕姿',
        '留出 G 实际工作姿态',
    ))
    def __init__(self,parent,id):  
        wx.Frame.__init__(self,parent,id,'Elfin E05 新手控制面板',pos=(120,60))
        # Keep the session log visible even when the controls are taller than
        # a laptop screen: controls scroll in the upper pane, while the log is
        # a separately resizable lower pane.
        self.workspace_splitter=wx.SplitterWindow(
            self, style=wx.SP_LIVE_UPDATE | wx.SP_3D)
        self.panel=wx.ScrolledWindow(
            self.workspace_splitter, style=wx.VSCROLL | wx.TAB_TRAVERSAL)
        self.panel.SetScrollRate(0, 12)
        self.log_panel=wx.Panel(self.workspace_splitter)
        font=self.panel.GetFont()
        font.SetPointSize(max(font.GetPointSize(), 9))
        self.panel.SetFont(font)
        self.log_panel.SetFont(font)
        self.main_sizer=wx.BoxSizer(wx.VERTICAL)
        frame_sizer=wx.BoxSizer(wx.VERTICAL)
        frame_sizer.Add(self.workspace_splitter, 1, wx.EXPAND)
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
        self.event_log_last={}
        self.event_log_lines=deque(maxlen=800)
        self.latest_event_issue=''
        self.validation_metrics={}
        self.recorded_point_seen=False
        self.payload_profile='等待末端负载管理器'
        self.payload_calibration_process=None
        self.payload_calibration_lock=threading.Lock()

        self.key=[]

        self.create_main_controls()
        self.display_init()

        self.servo_state=bool()
        self.servo_state_received=False
        self.servo_state_received_at=0.0
        self.servo_state_lock=threading.Lock()

        self.fault_state=bool()
        self.fault_state_received=False
        self.fault_state_received_at=0.0
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
        self.call_list_freedrive_points=rospy.ServiceProxy(
            '/elfin_freedrive_manager/list_recorded_points', ListRecordedPoints)
        self.call_delete_freedrive_point=rospy.ServiceProxy(
            '/elfin_freedrive_manager/delete_recorded_point', DeleteRecordedPoint)
        self.call_set_freedrive_velocity_scale=rospy.ServiceProxy(
            '/elfin_freedrive_manager/set_velocity_limit_scale', SetFloat64)
        self.call_set_freedrive_damping_scales=rospy.ServiceProxy(
            '/elfin_freedrive_manager/set_damping_scales', SetDampingScales)
        self.enter_freedrive_btn.Bind(
            wx.EVT_BUTTON, lambda evt: self.request_freedrive(evt, True))
        self.exit_freedrive_btn.Bind(
            wx.EVT_BUTTON, lambda evt: self.request_freedrive(evt, False))
        self.record_point_btn.Bind(wx.EVT_BUTTON, self.request_record_point)
        self.pose_manager_btn.Bind(wx.EVT_BUTTON, self.show_pose_manager_dialog)
        self.pose_refresh_btn.Bind(wx.EVT_BUTTON, self.request_pose_list)
        self.pose_select_all_btn.Bind(wx.EVT_BUTTON, self.select_all_poses)
        self.pose_delete_btn.Bind(wx.EVT_BUTTON, self.request_delete_pose)
        self.freedrive_speed_slider.Bind(
            wx.EVT_SLIDER, self.preview_freedrive_speed_scale)
        self.freedrive_speed_apply_btn.Bind(
            wx.EVT_BUTTON, self.request_freedrive_speed_scale)
        self.freedrive_damping_apply_btn.Bind(
            wx.EVT_BUTTON, self.request_freedrive_damping_scales)
        self.payload_calibration_start_btn.Bind(
            wx.EVT_BUTTON, self.request_payload_calibration)
        self.payload_calibration_resume_btn.Bind(
            wx.EVT_BUTTON,
            lambda evt: self.request_payload_calibration(evt, resume_samples=True))
        self.payload_calibration_cancel_btn.Bind(
            wx.EVT_BUTTON, self.cancel_payload_calibration)
                
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
        self.advanced_btn.Bind(wx.EVT_BUTTON, self.show_advanced_dialog)
        
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
        self.workspace_splitter.SplitHorizontally(
            self.panel, self.log_panel, sashPosition=-255)
        self.workspace_splitter.SetMinimumPaneSize(170)
        self.workspace_splitter.SetSashGravity(0.64)
        display_size=wx.GetDisplaySize()
        display_width=display_size.GetWidth()
        display_height=display_size.GetHeight()
        frame_width=min(1360, max(1050, display_width-24))
        frame_height=min(900, max(680, display_height-36))
        self.SetMinSize((min(1050, frame_width), min(680, frame_height)))
        self.SetSize((frame_width, frame_height))
        self.Centre()

    def make_description(self, parent, text, width=220):
        label=wx.StaticText(parent, label=text)
        label.Wrap(width)
        label.SetForegroundColour(wx.Colour(70, 70, 70))
        return label

    def make_indicator_grid(self, title, definitions, columns=3):
        """Build one cohesive, fixed-density instrument block."""
        box=wx.StaticBoxSizer(
            wx.StaticBox(self.panel, label=title), wx.VERTICAL)
        rows=(len(definitions)+columns-1)//columns
        grid=wx.FlexGridSizer(rows=rows, cols=columns, vgap=6, hgap=6)
        controls=[]
        for value, tooltip in definitions:
            control=wx.TextCtrl(
                self.panel,
                style=wx.TE_CENTER | wx.TE_MULTILINE | wx.TE_READONLY |
                      wx.TE_NO_VSCROLL,
                value=value)
            control.SetMinSize((108, 46))
            font=control.GetFont()
            font.SetWeight(wx.FONTWEIGHT_BOLD)
            control.SetFont(font)
            control.SetToolTip(tooltip)
            grid.Add(control, 0, wx.EXPAND)
            controls.append(control)
        for column in range(columns):
            grid.AddGrowableCol(column, 1)
        box.Add(grid, 0, wx.ALL | wx.EXPAND, 5)
        return box, controls

    def create_main_controls(self):
        header=wx.BoxSizer(wx.HORIZONTAL)
        title=wx.StaticText(self.panel, label='ELFIN E05  人工接管台')
        title_font=title.GetFont()
        title_font.SetPointSize(title_font.GetPointSize()+4)
        title_font.SetWeight(wx.FONTWEIGHT_BOLD)
        title.SetFont(title_font)
        title.SetToolTip('Elfin E05 真机/仿真共用人工控制面板')
        header.Add(title, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 16)
        header.AddStretchSpacer(1)

        self.set_links_btn=wx.Button(self.panel, label='坐标系')
        self.set_links_btn.SetToolTip('设置笛卡尔点动使用的参考坐标系和末端连杆')
        self.pose_manager_btn=wx.Button(self.panel, label='姿态管理')
        self.pose_manager_btn.SetToolTip('逐条查看、记录和删除 POINT 姿态')
        self.advanced_btn=wx.Button(self.panel, label='拖拽高级')
        self.advanced_btn.SetToolTip('设置六轴阻尼并查看完整预检和试验文件')
        self.maintenance_btn=wx.Button(self.panel, label='维护')
        self.maintenance_btn.SetToolTip('危险操作：三组抱闸控制和底层接口诊断')
        for button in (
                self.set_links_btn, self.pose_manager_btn,
                self.advanced_btn, self.maintenance_btn):
            header.Add(button, 0, wx.LEFT, 5)
        self.main_sizer.Add(header, 0, wx.ALL | wx.EXPAND, 7)

        monitor_defs=(
            ('SERVO\n等待', '六轴伺服使能状态；Servo On 不代表无 Fault。'),
            ('FAULT\n等待', '底层驱动故障锁存；触发时先排除机械和供电原因。'),
            ('FREE\n等待', 'READY 可进入，ACTIVE 表示重力补偿控制器占用六轴。'),
            ('关节反馈\n等待', 'joint_states 六轴反馈是否已收到；过期时不能安全进入 FREE。'),
            ('I/O 服务\n等待', '末端数字 I/O read_di/read_do 服务在线状态。'),
            ('INPUT_0 / DI0\n未知', '末端 INPUT_0 / DI0，只读；PNP 11-30 V 为 ON。'),
            ('INPUT_1 / DI1\n未知', '末端 INPUT_1 / DI1，只读；PNP 11-30 V 为 ON。'),
            ('INPUT_2 / DI2\n未知', '末端 INPUT_2 / DI2，只读；PNP 11-30 V 为 ON。'),
            ('姿态记录\n0 条', 'POINT 持久记录数量；点击“姿态管理”查看明细。'),
            ('POINT / bit 4\n等待', '实体 POINT 固定为 DI bit 4；按下沿记录当前姿态。'),
            ('FREE / bit 5\n等待', '实体 FREE 固定为 DI bit 5；按下进入，松开退出。'),
            ('灯环\n等待', '厂家手册预期颜色；当前驱动不主动写实体灯环。'))
        self.monitor_status_box, monitor_controls=self.make_indicator_grid(
            '机器人状态与末端输入', monitor_defs, columns=4)
        (self.servo_state_show, self.fault_state_show,
         self.freedrive_state_show, self.joint_state_show,
         self.io_status_show, self.DI_display[0], self.DI_display[1],
         self.DI_display[2], self.freedrive_point_count_show,
         point_button_show, free_button_show,
         self.freedrive_ring_show)=monitor_controls
        self.tool_button_state_show={
            'POINT': point_button_show,
            'FREE': free_button_show}

        self.monitor_detail_box=wx.StaticBoxSizer(
            wx.StaticBox(self.panel, label='FREE 管理器当前详情'),
            wx.VERTICAL)
        self.freedrive_detail_show=wx.TextCtrl(
            self.panel,
            style=wx.TE_MULTILINE | wx.TE_READONLY |
                  getattr(wx, 'TE_CHARWRAP', wx.TE_WORDWRAP),
            value='等待 elfin_freedrive_manager')
        self.freedrive_detail_show.SetMinSize((-1, 42))
        self.freedrive_detail_show.SetToolTip(
            '完整切换进度、拒绝、保护回退和被动退出原因；同时追加到事件日志')
        self.monitor_detail_box.Add(
            self.freedrive_detail_show, 0, wx.ALL | wx.EXPAND, 5)

        self.monitor_validation_box=wx.StaticBoxSizer(
            wx.StaticBox(self.panel, label='重力模型预检'), wx.VERTICAL)
        validation_grid=wx.FlexGridSizer(rows=2, cols=4, vgap=4, hgap=4)
        metric_defs=(
            ('result', '结论', '等待', '预检总结果：通过、警告允许进入、未通过或等待。'),
            ('excited', '有效轴', '--', '参与反馈/重力模型比较且力矩足够的关节数量。'),
            ('reverse', '反向轴', '--', '反馈力矩方向与重力模型相反的关节数量。'),
            ('alignment', '一致度', '--', '反馈与模型方向一致度，越接近 1 越一致。'),
            ('scale', '比例', '--', '静态反馈力矩相对模型力矩的整体比例估计。'),
            ('residual', '残差', '--', '去除整体比例后的归一化误差，越小越好。'),
            ('stddev', '波动 Nm', '--', '静止采样期间有效轴反馈力矩的最大标准差。'),
            ('capacity', '力矩容量', '--', '当前重力模型是否仍在配置力矩容量内。'))
        self.validation_metric_shows={}
        for key, label, value, tooltip in metric_defs:
            control=wx.TextCtrl(
                self.panel,
                style=wx.TE_CENTER | wx.TE_MULTILINE | wx.TE_READONLY |
                      wx.TE_NO_VSCROLL,
                value=label+'\n'+value)
            control.SetMinSize((92, 43))
            control.SetToolTip(tooltip)
            validation_grid.Add(control, 1, wx.EXPAND)
            self.validation_metric_shows[key]=control
        self.validation_metric_labels={
            key: label for key, label, value, tooltip in metric_defs}
        for column in range(4):
            validation_grid.AddGrowableCol(column, 1)
        self.monitor_validation_box.Add(
            validation_grid, 0, wx.ALL | wx.EXPAND, 5)

    def build_motion_box(self, title, labels, displays, minus_buttons,
                         plus_buttons, callback, descriptions):
        box=wx.StaticBoxSizer(wx.StaticBox(self.panel, label=title), wx.VERTICAL)
        grid=wx.FlexGridSizer(rows=7, cols=4, vgap=3, hgap=4)
        for heading in ('轴', '－', '实时值', '＋'):
            text=wx.StaticText(self.panel, label=heading)
            text.SetFont(text.GetFont().Bold())
            grid.Add(text, 0, wx.ALIGN_CENTER)
        for index, (label, description) in enumerate(zip(labels, descriptions)):
            axis=wx.StaticText(self.panel, label=label)
            axis.SetToolTip(description)
            grid.Add(axis, 0, wx.ALIGN_CENTER_VERTICAL)
            minus=wx.Button(self.panel, label='－', size=(38, 38))
            minus.SetToolTip('按住：'+description+' 负方向；松开立即请求 Stop')
            minus.Bind(
                wx.EVT_LEFT_DOWN,
                lambda evt, mark=-(index+1), cb=callback: cb(evt, mark))
            minus.Bind(
                wx.EVT_LEFT_UP,
                lambda evt, mark=-(index+1): self.release_button(evt, mark))
            minus_buttons[index]=minus
            grid.Add(minus, 0, wx.EXPAND)
            value=wx.TextCtrl(
                self.panel, style=wx.TE_CENTER | wx.TE_READONLY,
                value='--', size=(88, 34))
            value.SetToolTip(description+' 当前反馈值')
            displays[index]=value
            grid.Add(value, 0, wx.EXPAND | wx.ALIGN_CENTER_VERTICAL)
            plus=wx.Button(self.panel, label='＋', size=(38, 38))
            plus.SetToolTip('按住：'+description+' 正方向；松开立即请求 Stop')
            plus.Bind(
                wx.EVT_LEFT_DOWN,
                lambda evt, mark=index+1, cb=callback: cb(evt, mark))
            plus.Bind(
                wx.EVT_LEFT_UP,
                lambda evt, mark=index+1: self.release_button(evt, mark))
            plus_buttons[index]=plus
            grid.Add(plus, 0, wx.EXPAND)
        box.Add(grid, 1, wx.ALL | wx.EXPAND, 5)
        box.SetMinSize((240, -1))
        return box

    def display_init(self):
        top_row=wx.BoxSizer(wx.HORIZONTAL)
        joint_descriptions=(
            'J1 基座旋转', 'J2 肩关节俯仰', 'J3 肘关节折叠',
            'J4 前臂/腕部旋转', 'J5 腕部俯仰', 'J6 工具法兰旋转')
        joint_labels=tuple('J'+str(index+1)+' / deg' for index in range(6))
        joint_box=self.build_motion_box(
            '六关节点动', joint_labels, self.js_display,
            self.jm_button, self.jp_button, self.teleop_joints,
            joint_descriptions)
        top_row.Add(joint_box, 0, wx.RIGHT, 6)

        cart_labels=('X / mm', 'Y / mm', 'Z / mm',
                     'Rx / deg', 'Ry / deg', 'Rz / deg')
        cart_descriptions=(
            '末端沿参考坐标系 X 轴平移',
            '末端沿参考坐标系 Y 轴平移',
            '末端沿参考坐标系 Z 轴平移',
            '末端绕参考坐标系 X 轴连续旋转',
            '末端绕参考坐标系 Y 轴连续旋转',
            '末端绕参考坐标系 Z 轴连续旋转')
        cart_box=self.build_motion_box(
            '六维末端点动', cart_labels, self.ps_display,
            self.pm_button, self.pp_button, self.teleop_pcs,
            cart_descriptions)
        top_row.Add(cart_box, 0, wx.RIGHT, 6)

        control_box=wx.StaticBoxSizer(
            wx.StaticBox(self.panel, label='接管与输出'), wx.VERTICAL)
        control_box.SetMinSize((335, -1))
        self.stop_btn=wx.Button(
            self.panel, label='STOP  停止当前轨迹', name='Stop')
        self.stop_btn.SetMinSize((-1, 42))
        self.stop_btn.SetBackgroundColour(wx.Colour(205, 70, 60))
        self.stop_btn.SetForegroundColour(wx.WHITE)
        self.stop_btn.SetToolTip(
            '取消 Panel 点动或当前轨迹并保持 Servo 状态；不是物理断电急停')
        control_box.Add(self.stop_btn, 0, wx.ALL | wx.EXPAND, 5)

        command_grid=wx.GridSizer(rows=1, cols=4, vgap=4, hgap=4)
        command_defs=(
            ('power_on_btn', 'Servo On', 'Servo On',
             '位置对齐后使能六轴并启动轨迹控制器。'),
            ('power_off_btn', 'Servo Off', 'Servo Off',
             '取消轨迹并关闭六轴伺服。'),
            ('reset_btn', '清 Fault', 'Clear Fault',
             '只清故障锁存，不消除碰撞、过载或供电原因。'),
            ('home_btn', '按住 Home', 'home_btn',
             '按住运动到固定 ROS 六轴零位；松开 Stop。'))
        for attribute, label, name, tooltip in command_defs:
            button=wx.Button(self.panel, label=label, name=name)
            button.SetMinSize((78, 44))
            button.SetToolTip(tooltip)
            setattr(self, attribute, button)
            command_grid.Add(button, 1, wx.EXPAND)
        control_box.Add(command_grid, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 5)

        velocity_scaling_init=rospy.get_param(
            self.elfin_basic_api_ns+'velocity_scaling', default=0.01)
        point_row=wx.BoxSizer(wx.HORIZONTAL)
        point_label=wx.StaticText(self.panel, label='点动')
        point_label.SetToolTip('只影响 Panel/Basic API 点动；不影响 RViz MoveIt')
        point_row.Add(point_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)
        self.velocity_setting=wx.Slider(
            self.panel, value=int(velocity_scaling_init*100),
            minValue=1, maxValue=100, style=wx.SL_HORIZONTAL)
        self.velocity_setting.SetToolTip(
            'Panel 关节/笛卡尔点动速度倍率，范围 1% 到 100%')
        point_row.Add(self.velocity_setting, 1, wx.ALIGN_CENTER_VERTICAL)
        self.velocity_setting_show=wx.TextCtrl(
            self.panel, style=wx.TE_CENTER | wx.TE_READONLY,
            value=str(round(velocity_scaling_init*100, 1))+'%', size=(58, 26))
        self.velocity_setting_show.SetToolTip('当前 Panel 点动速度倍率')
        point_row.Add(self.velocity_setting_show, 0, wx.LEFT, 4)
        self.velocity_setting.Bind(wx.EVT_SLIDER, self.velocity_setting_cb)
        control_box.Add(point_row, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 5)

        free_row=wx.BoxSizer(wx.HORIZONTAL)
        self.enter_freedrive_btn=wx.Button(self.panel, label='进入 FREE')
        self.enter_freedrive_btn.SetMinSize((-1, 42))
        self.enter_freedrive_btn.SetToolTip(
            '软件进入后无限持续，直到人工退出，或实体 FREE 完成一次按下再松开')
        self.exit_freedrive_btn=wx.Button(self.panel, label='退出并保持')
        self.exit_freedrive_btn.SetMinSize((-1, 42))
        self.exit_freedrive_btn.SetToolTip(
            '退出重力补偿，确认静止后恢复当前位置保持')
        self.exit_freedrive_btn.Disable()
        free_row.Add(self.enter_freedrive_btn, 1, wx.RIGHT | wx.EXPAND, 4)
        free_row.Add(self.exit_freedrive_btn, 1, wx.EXPAND)
        control_box.Add(free_row, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 5)

        free_speed_row=wx.BoxSizer(wx.HORIZONTAL)
        free_speed_label=wx.StaticText(self.panel, label='FREE 上限')
        free_speed_label.SetToolTip(
            '缩放拖拽软减速和超速退出阈值；不改变关节角限位、力矩或 Fault 保护')
        free_speed_row.Add(
            free_speed_label, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)
        self.freedrive_speed_slider=wx.Slider(
            self.panel, value=100, minValue=50, maxValue=300,
            style=wx.SL_HORIZONTAL)
        self.freedrive_speed_slider.SetToolTip(
            '拖拽速度保护倍率 50% 到 300%；退出 FREE 后应用')
        free_speed_row.Add(
            self.freedrive_speed_slider, 1, wx.ALIGN_CENTER_VERTICAL)
        self.freedrive_speed_apply_btn=wx.Button(
            self.panel, label='应用 100%', size=(82, -1))
        self.freedrive_speed_apply_btn.SetToolTip(
            '只在零力控制器未运行时应用，下一次进入 FREE 生效')
        free_speed_row.Add(self.freedrive_speed_apply_btn, 0, wx.LEFT, 4)
        control_box.Add(
            free_speed_row, 0,
            wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 5)
        output_box=wx.StaticBoxSizer(
            wx.StaticBox(self.panel, label='末端手动输出（OUTPUT_0..2）'),
            wx.VERTICAL)
        do_row=wx.BoxSizer(wx.HORIZONTAL)
        for index in range(3):
            button=wx.Button(
                self.panel, label='OUTPUT_'+str(index)+'\n未知')
            button.SetMinSize((-1, 40))
            button.SetToolTip(
                '点击切换 OUTPUT_'+str(index)+
                '，写入后回读；接负载前先确认电气极性和电流')
            button.Bind(
                wx.EVT_BUTTON,
                lambda evt, marker=index, cl=self.call_write_DO:
                self.call_write_DO_command(evt, marker, cl))
            self.DO_btn_display[index]=button
            do_row.Add(button, 1, wx.RIGHT if index < 2 else 0, 3)
        output_box.Add(do_row, 0, wx.ALL | wx.EXPAND, 4)
        control_box.Add(
            output_box, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 5)

        link_grid=wx.FlexGridSizer(rows=1, cols=4, vgap=3, hgap=4)
        link_grid.Add(wx.StaticText(self.panel, label='Ref'), 0, wx.ALIGN_CENTER_VERTICAL)
        self.ref_link_show=wx.TextCtrl(
            self.panel, style=wx.TE_READONLY, value=self.ref_link_name,
            size=(-1, 24))
        self.ref_link_show.SetToolTip('当前笛卡尔点动参考坐标系')
        link_grid.Add(self.ref_link_show, 1, wx.EXPAND)
        link_grid.Add(wx.StaticText(self.panel, label='Tool'), 0, wx.ALIGN_CENTER_VERTICAL)
        self.end_link_show=wx.TextCtrl(
            self.panel, style=wx.TE_READONLY, value=self.end_link_name,
            size=(-1, 24))
        self.end_link_show.SetToolTip('当前笛卡尔点动末端连杆')
        link_grid.Add(self.end_link_show, 1, wx.EXPAND)
        link_grid.AddGrowableCol(1, 1)
        link_grid.AddGrowableCol(3, 1)
        control_box.Add(
            link_grid, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 5)

        top_row.Add(control_box, 0, wx.RIGHT, 6)

        monitor_column=wx.BoxSizer(wx.VERTICAL)
        monitor_column.Add(
            self.monitor_status_box, 0, wx.BOTTOM | wx.EXPAND, 5)
        monitor_column.Add(
            self.monitor_detail_box, 0, wx.BOTTOM | wx.EXPAND, 5)
        monitor_column.Add(
            self.monitor_validation_box, 0, wx.EXPAND)
        top_row.Add(monitor_column, 1, wx.EXPAND)

        log_box=wx.StaticBoxSizer(
            wx.StaticBox(self.log_panel, label='会话事件日志'),
            wx.VERTICAL)
        log_toolbar=wx.BoxSizer(wx.HORIZONTAL)
        self.follow_log_check=wx.CheckBox(self.log_panel, label='跟随最新')
        self.follow_log_check.SetValue(True)
        self.follow_log_check.SetToolTip(
            '点击日志、滚动或选择文字会自动暂停；恢复后才滚到末尾')
        self.follow_log_state=wx.StaticText(self.log_panel, label='实时跟随')
        self.follow_log_check.Bind(wx.EVT_CHECKBOX, self.follow_event_log)
        log_toolbar.Add(
            self.follow_log_check, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 7)
        log_toolbar.Add(self.follow_log_state, 0, wx.ALIGN_CENTER_VERTICAL)
        log_toolbar.AddStretchSpacer(1)
        locate_log_btn=wx.Button(
            self.log_panel, label='定位最近异常', size=(108, -1))
        locate_log_btn.SetToolTip('选中并显示最近一条警告或错误，不会自动恢复跟随')
        locate_log_btn.Bind(wx.EVT_BUTTON, self.locate_latest_event_issue)
        self.locate_log_btn=locate_log_btn
        self.locate_log_btn.Enable(False)
        clear_log_btn=wx.Button(self.log_panel, label='清空', size=(64, -1))
        clear_log_btn.SetToolTip('只清空本次界面日志，不删除 ROS 日志或拖拽 CSV')
        clear_log_btn.Bind(wx.EVT_BUTTON, self.clear_event_log)
        copy_log_btn=wx.Button(self.log_panel, label='复制选中/全部', size=(112, -1))
        copy_log_btn.SetToolTip('有选区时只复制选区；否则复制当前全部会话日志')
        copy_log_btn.Bind(wx.EVT_BUTTON, self.copy_event_log)
        log_toolbar.Add(locate_log_btn, 0, wx.LEFT, 4)
        log_toolbar.Add(clear_log_btn, 0, wx.LEFT, 4)
        log_toolbar.Add(copy_log_btn, 0, wx.LEFT, 4)
        log_box.Add(log_toolbar, 0, wx.LEFT | wx.RIGHT | wx.TOP | wx.EXPAND, 5)

        self.latest_event_issue_show=wx.TextCtrl(
            self.log_panel, value='最近警告/错误：暂无',
            style=wx.TE_MULTILINE | wx.TE_READONLY |
                  getattr(wx, 'TE_CHARWRAP', wx.TE_WORDWRAP) |
                  wx.BORDER_SIMPLE)
        self.latest_event_issue_show.SetMinSize((-1, 42))
        self.latest_event_issue_show.SetMaxSize((-1, 54))
        self.latest_event_issue_show.SetBackgroundColour(wx.Colour(255, 244, 228))
        log_box.Add(
            self.latest_event_issue_show, 0,
            wx.LEFT | wx.RIGHT | wx.TOP | wx.EXPAND, 5)
        self.reply_show=wx.TextCtrl(
            self.log_panel,
            style=wx.TE_MULTILINE | wx.TE_READONLY |
                  getattr(wx, 'TE_CHARWRAP', wx.TE_WORDWRAP) |
                  wx.TE_RICH2,
            value='')
        self.reply_show.SetMinSize((390, 105))
        self.reply_show.SetToolTip(
            '按框宽逐字符换行；点击、滚动或选择会暂停自动跟随，复制期间选区不会跳走')
        self.reply_show.Bind(wx.EVT_LEFT_DOWN, self.pause_event_log)
        self.reply_show.Bind(wx.EVT_MOUSEWHEEL, self.pause_event_log)
        self.reply_show.Bind(wx.EVT_KEY_DOWN, self.pause_event_log)
        log_box.Add(self.reply_show, 1, wx.ALL | wx.EXPAND, 5)
        self.log_panel.SetSizer(log_box)
        self.main_sizer.Add(
            top_row, 1, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 7)

        self.init_pose_manager_dialog()
        self.init_advanced_dialog()
        self.append_event('系统', '控制面板已启动，正在读取 ROS 状态。')

    def init_pose_manager_dialog(self):
        self.pose_manager_dlg=wx.Dialog(
            self, title='POINT 姿态管理',
            style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
        self.pose_manager_dlg.Bind(
            wx.EVT_CLOSE, lambda event: self.hide_aux_dialog(
                event, self.pose_manager_dlg))
        root=wx.BoxSizer(wx.VERTICAL)
        path_row=wx.BoxSizer(wx.HORIZONTAL)
        path_row.Add(
            wx.StaticText(self.pose_manager_dlg, label='记录文件'),
            0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 6)
        self.pose_record_file_show=wx.TextCtrl(
            self.pose_manager_dlg, style=wx.TE_READONLY,
            value='等待管理节点返回路径')
        self.pose_record_file_show.SetToolTip(
            '管理节点实际使用的 YAML 文件；默认是 ~/.ros/elfin_freedrive_points.yaml')
        path_row.Add(self.pose_record_file_show, 1, wx.RIGHT | wx.EXPAND, 5)
        copy_path=wx.Button(self.pose_manager_dlg, label='复制路径')
        copy_path.SetToolTip('复制完整姿态 YAML 路径')
        copy_path.Bind(wx.EVT_BUTTON, self.copy_pose_path)
        path_row.Add(copy_path, 0)
        root.Add(path_row, 0, wx.ALL | wx.EXPAND, 8)

        self.pose_list=wx.ListCtrl(
            self.pose_manager_dlg,
            style=wx.LC_REPORT | wx.BORDER_SUNKEN)
        columns=(
            ('序号', 55), ('时间', 145), ('来源', 155),
            ('J1°', 75), ('J2°', 75), ('J3°', 75),
            ('J4°', 75), ('J5°', 75), ('J6°', 75))
        for index, (label, width) in enumerate(columns):
            self.pose_list.InsertColumn(index, label, width=width)
        self.pose_list.SetToolTip(
            '每行是一个离散 POINT 姿态；角度仅展示，文件中保存弧度')
        self.pose_delete_in_progress=False
        self.pose_list.Bind(
            wx.EVT_LIST_ITEM_SELECTED, self.update_pose_selection_controls)
        self.pose_list.Bind(
            wx.EVT_LIST_ITEM_DESELECTED, self.update_pose_selection_controls)
        root.Add(self.pose_list, 1, wx.LEFT | wx.RIGHT | wx.EXPAND, 8)

        buttons=wx.BoxSizer(wx.HORIZONTAL)
        self.pose_refresh_btn=wx.Button(self.pose_manager_dlg, label='刷新列表')
        self.pose_refresh_btn.SetToolTip('从管理节点重新读取姿态 YAML')
        self.record_point_btn=wx.Button(
            self.pose_manager_dlg, label='记录当前姿态')
        self.record_point_btn.SetToolTip(
            '追加当前六轴反馈姿态；不移动机器人')
        self.pose_select_all_btn=wx.Button(
            self.pose_manager_dlg, label='全选')
        self.pose_select_all_btn.SetToolTip('选中列表中的全部 POINT 姿态')
        self.pose_select_all_btn.Disable()
        self.pose_delete_btn=wx.Button(
            self.pose_manager_dlg, label='删除选中')
        self.pose_delete_btn.SetToolTip(
            '一次删除全部选中姿态；全选时由管理节点原子清空 YAML')
        self.pose_delete_btn.Disable()
        close=wx.Button(self.pose_manager_dlg, wx.ID_CLOSE, label='关闭')
        close.Bind(
            wx.EVT_BUTTON,
            lambda event: self.hide_aux_dialog(event, self.pose_manager_dlg))
        for button in (
                self.pose_refresh_btn, self.record_point_btn,
                self.pose_select_all_btn,
                self.pose_delete_btn):
            buttons.Add(button, 0, wx.RIGHT, 5)
        buttons.AddStretchSpacer(1)
        buttons.Add(close, 0)
        root.Add(buttons, 0, wx.ALL | wx.EXPAND, 8)
        self.pose_manager_dlg.SetSizer(root)
        self.pose_manager_dlg.SetSize((900, 480))
        self.pose_manager_dlg.SetMinSize((760, 360))
        self.pose_manager_dlg.Layout()

    def init_advanced_dialog(self):
        self.advanced_dlg=wx.Dialog(
            self, title='拖拽高级设置与诊断',
            style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER)
        self.advanced_dlg.Bind(
            wx.EVT_CLOSE,
            lambda event: self.hide_aux_dialog(event, self.advanced_dlg))
        root=wx.BoxSizer(wx.VERTICAL)

        payload_box=wx.StaticBoxSizer(
            wx.StaticBox(self.advanced_dlg, label='未知末端负载自动标定'),
            wx.VERTICAL)
        self.payload_profile_show=wx.TextCtrl(
            self.advanced_dlg,
            style=wx.TE_MULTILINE | wx.TE_READONLY |
                  getattr(wx, 'TE_CHARWRAP', wx.TE_WORDWRAP),
            value=self.payload_profile)
        self.payload_profile_show.SetMinSize((-1, 68))
        self.payload_profile_show.SetToolTip(
            '当前实际启用的质量、法兰坐标系三维重心、验证指标和持久化文件')
        payload_box.Add(
            self.payload_profile_show, 0, wx.ALL | wx.EXPAND, 6)
        payload_buttons=wx.BoxSizer(wx.HORIZONTAL)
        self.payload_calibration_start_btn=wx.Button(
            self.advanced_dlg, label='高位一键标定未知末端')
        self.payload_calibration_start_btn.SetToolTip(
            '要求法兰高于 0.45 m；以 5% 位置控制速度走 5 个拟合和 2 个留出姿态；'
            '每段都复核 MoveIt、全臂 z=0 与当前 STEP 包络 z=0.30 m 门禁')
        self.payload_calibration_resume_btn=wx.Button(
            self.advanced_dlg, label='复用最近样本，仅做短时验证')
        self.payload_calibration_resume_btn.SetToolTip(
            '只接受当前 7 姿态版本的完整样本；末端和线缆必须保持不变；'
            '低位时先以 5% 经 MoveIt 抬到实际工作姿态，再做最多 1 秒保持')
        self.payload_calibration_cancel_btn=wx.Button(
            self.advanced_dlg, label='中止标定')
        self.payload_calibration_cancel_btn.SetToolTip(
            '请求脚本停止当前轨迹、退出 FREE、回滚候选模型并恢复原阻尼')
        self.payload_calibration_cancel_btn.Disable()
        payload_buttons.Add(self.payload_calibration_start_btn, 1, wx.RIGHT, 6)
        payload_buttons.Add(self.payload_calibration_resume_btn, 1, wx.RIGHT, 6)
        payload_buttons.Add(self.payload_calibration_cancel_btn, 0)
        payload_box.Add(
            payload_buttons, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 6)
        self.payload_calibration_output=wx.TextCtrl(
            self.advanced_dlg,
            style=wx.TE_MULTILINE | wx.TE_READONLY |
                  getattr(wx, 'TE_CHARWRAP', wx.TE_WORDWRAP),
            value='尚未启动自动负载标定。')
        self.payload_calibration_output.SetMinSize((-1, 92))
        self.payload_calibration_output.SetToolTip(
            '按框宽换行；选中文字时新输出会保留选区和滚动位置；同样追加到主会话日志')
        payload_box.Add(
            self.payload_calibration_output, 1,
            wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 6)
        root.Add(payload_box, 1, wx.ALL | wx.EXPAND, 8)

        speed_box=wx.StaticBoxSizer(
            wx.StaticBox(self.advanced_dlg, label='FREE 速度保护诊断'),
            wx.VERTICAL)
        self.freedrive_speed_state_show=wx.TextCtrl(
            self.advanced_dlg,
            style=wx.TE_MULTILINE | wx.TE_READONLY |
                  getattr(wx, 'TE_CHARWRAP', wx.TE_WORDWRAP),
            value='当前 100%；等待六轴硬上限')
        self.freedrive_speed_state_show.SetMinSize((-1, 58))
        self.freedrive_speed_state_show.SetToolTip(
            '主界面只保留百分比；这里完整显示 J1-J6 换算后的诊断阈值')
        speed_box.Add(
            self.freedrive_speed_state_show, 1, wx.ALL | wx.EXPAND, 6)
        root.Add(speed_box, 0, wx.ALL | wx.EXPAND, 8)

        damping_box=wx.StaticBoxSizer(
            wx.StaticBox(self.advanced_dlg, label='六轴速度阻尼倍率'),
            wx.VERTICAL)
        self.freedrive_damping_state_show=wx.TextCtrl(
            self.advanced_dlg, style=wx.TE_READONLY,
            value='当前：J1-J6 均为 100%')
        self.freedrive_damping_state_show.SetToolTip(
            '控制器实际采用的逐轴速度阻尼倍率')
        damping_box.Add(
            self.freedrive_damping_state_show, 0,
            wx.ALL | wx.EXPAND, 6)
        damping_row=wx.BoxSizer(wx.HORIZONTAL)
        self.freedrive_damping_inputs=[]
        for index in range(6):
            column=wx.BoxSizer(wx.VERTICAL)
            label=wx.StaticText(self.advanced_dlg, label='J'+str(index+1))
            label.SetToolTip(
                'J'+str(index+1)+' 阻尼倍率；越低越轻，越高越稳')
            column.Add(label, 0, wx.ALIGN_CENTER_HORIZONTAL | wx.BOTTOM, 2)
            control=wx.SpinCtrlDouble(
                self.advanced_dlg, min=0.05, max=5.0,
                initial=1.0, inc=0.05, style=wx.SP_ARROW_KEYS)
            control.SetDigits(2)
            control.SetMinSize((82, -1))
            control.SetToolTip(
                '0.05 到 5.00；此值确实参与控制器 command = gravity - damping * scale * velocity')
            self.freedrive_damping_inputs.append(control)
            column.Add(control, 0, wx.EXPAND)
            damping_row.Add(column, 1, wx.RIGHT if index < 5 else 0, 5)
        damping_box.Add(
            damping_row, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 6)
        self.freedrive_damping_apply_btn=wx.Button(
            self.advanced_dlg, label='应用六轴阻尼')
        self.freedrive_damping_apply_btn.SetToolTip(
            '只在 FREE 未运行时应用；下一次进入生效')
        damping_box.Add(
            self.freedrive_damping_apply_btn, 0,
            wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 6)
        root.Add(damping_box, 0, wx.ALL | wx.EXPAND, 8)

        validation_box=wx.StaticBoxSizer(
            wx.StaticBox(self.advanced_dlg, label='完整重力模型预检'),
            wx.VERTICAL)
        self.freedrive_validation_show=wx.TextCtrl(
            self.advanced_dlg,
            style=wx.TE_MULTILINE | wx.TE_READONLY |
                  getattr(wx, 'TE_CHARWRAP', wx.TE_WORDWRAP),
            value='等待重力模型预检')
        self.freedrive_validation_show.SetMinSize((-1, 85))
        self.freedrive_validation_show.SetToolTip(
            '主界面指标的完整原始文本，含当前姿态力矩容量详情')
        validation_box.Add(
            self.freedrive_validation_show, 1, wx.ALL | wx.EXPAND, 6)
        root.Add(
            validation_box, 1, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 8)

        log_box=wx.StaticBoxSizer(
            wx.StaticBox(self.advanced_dlg, label='本次拖拽 CSV'), wx.VERTICAL)
        self.freedrive_trial_log_show=wx.TextCtrl(
            self.advanced_dlg,
            style=wx.TE_MULTILINE | wx.TE_READONLY |
                  getattr(wx, 'TE_CHARWRAP', wx.TE_WORDWRAP),
            value='尚未开始本次试验')
        self.freedrive_trial_log_show.SetMinSize((-1, 42))
        self.freedrive_trial_log_show.SetToolTip(
            '本次 FREE 的逐周期位置、速度、力矩与退出事件 CSV 路径')
        log_box.Add(
            self.freedrive_trial_log_show, 0, wx.ALL | wx.EXPAND, 6)
        root.Add(log_box, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM | wx.EXPAND, 8)

        close=wx.Button(self.advanced_dlg, wx.ID_CLOSE, label='关闭')
        close.Bind(
            wx.EVT_BUTTON,
            lambda event: self.hide_aux_dialog(event, self.advanced_dlg))
        root.Add(close, 0, wx.RIGHT | wx.BOTTOM | wx.ALIGN_RIGHT, 8)
        self.advanced_dlg.SetSizer(root)
        self.advanced_dlg.SetSize((800, 720))
        self.advanced_dlg.SetMinSize((700, 620))
        self.advanced_dlg.Layout()

    def pause_event_log(self, event=None):
        if hasattr(self, 'follow_log_check'):
            self.follow_log_check.SetValue(False)
            self.follow_log_state.SetLabel('已暂停，选区受保护')
        if event is not None:
            event.Skip()

    def follow_event_log(self, event=None):
        if self.follow_log_check.GetValue():
            self.follow_log_state.SetLabel('实时跟随')
            self.reply_show.SetInsertionPointEnd()
            self.reply_show.ShowPosition(self.reply_show.GetLastPosition())
        else:
            self.follow_log_state.SetLabel('已暂停，选区受保护')
        if event is not None:
            event.Skip()

    def locate_latest_event_issue(self, event=None):
        if self.latest_event_issue:
            start=self.reply_show.GetValue().rfind(self.latest_event_issue)
            if start >= 0:
                self.pause_event_log()
                self.reply_show.SetFocus()
                self.reply_show.SetSelection(
                    start, start+len(self.latest_event_issue))
                self.reply_show.ShowPosition(start)
        if event is not None:
            event.Skip()

    def append_event(self, category, message, level='INFO', dedup_key=None):
        message=str(message).strip()
        if not message:
            return
        if dedup_key is not None:
            if self.event_log_last.get(dedup_key) == message:
                return
            self.event_log_last[dedup_key]=message
        if not hasattr(self, 'reply_show'):
            return
        stamp=time.strftime('%H:%M:%S')
        severity={'ERROR': '错误 · ', 'WARN': '警告 · '}.get(level, '')
        line='['+stamp+'] '+severity+str(category)+'：'+message
        self.event_log_lines.append(line)
        selection=self.reply_show.GetSelection()
        scroll=self.reply_show.GetScrollPos(wx.VERTICAL)
        self.reply_show.AppendText(line+'\n')
        following=(hasattr(self, 'follow_log_check') and
                   self.follow_log_check.GetValue())
        if following and selection[0] == selection[1]:
            self.reply_show.SetInsertionPointEnd()
            self.reply_show.ShowPosition(self.reply_show.GetLastPosition())
        else:
            end=self.reply_show.GetLastPosition()
            self.reply_show.SetSelection(
                min(selection[0], end), min(selection[1], end))
            self.reply_show.SetScrollPos(wx.VERTICAL, scroll)
        if level in ('WARN', 'ERROR'):
            self.latest_event_issue=line
            self.latest_event_issue_show.SetValue('最近警告/错误：'+line)
            self.locate_log_btn.Enable(True)

    def clear_event_log(self, event=None):
        self.reply_show.Clear()
        self.event_log_lines.clear()
        self.event_log_last.clear()
        self.latest_event_issue=''
        self.latest_event_issue_show.SetValue('最近警告/错误：暂无')
        self.locate_log_btn.Enable(False)
        self.append_event('系统', '会话日志已由操作者清空。')
        if event is not None:
            event.Skip()

    def copy_event_log(self, event=None):
        start,end=self.reply_show.GetSelection()
        value=(self.reply_show.GetRange(start,end)
               if start != end else self.reply_show.GetValue())
        if wx.TheClipboard.Open():
            wx.TheClipboard.SetData(wx.TextDataObject(value))
            wx.TheClipboard.Close()
            self.pause_event_log()
            self.follow_log_state.SetLabel('已复制；保持暂停')
        else:
            self.pause_event_log()
            self.follow_log_state.SetLabel('剪贴板不可用；保持暂停')
        if event is not None:
            event.Skip()

    def copy_pose_path(self, event=None):
        path=self.pose_record_file_show.GetValue()
        if wx.TheClipboard.Open():
            wx.TheClipboard.SetData(wx.TextDataObject(path))
            wx.TheClipboard.Close()
            self.append_event('姿态', '姿态文件路径已复制：'+path)
        else:
            self.append_event('姿态', '无法打开剪贴板。', 'ERROR')
        if event is not None:
            event.Skip()

    def hide_aux_dialog(self, event, dialog):
        dialog.Hide()
        if event is not None:
            event.Skip(False)

    def show_pose_manager_dialog(self, event=None):
        self.pose_manager_dlg.CentreOnParent()
        self.pose_manager_dlg.Layout()
        self.pose_manager_dlg.Show()
        self.pose_manager_dlg.Refresh()
        self.pose_manager_dlg.Update()
        self.pose_manager_dlg.Raise()
        self.request_pose_list()
        if event is not None:
            event.Skip()

    def show_advanced_dialog(self, event=None):
        self.advanced_dlg.CentreOnParent()
        self.advanced_dlg.Layout()
        self.advanced_dlg.Show()
        self.advanced_dlg.Refresh()
        self.advanced_dlg.Update()
        self.advanced_dlg.Raise()
        if event is not None:
            event.Skip()

    @staticmethod
    def latest_payload_sample_path():
        pattern=os.path.expanduser(
            '~/.ros/elfin_payload_calibration_runs/*/static_pairs.csv')
        candidates=sorted(
            (path for path in glob.glob(pattern) if os.path.isfile(path)),
            key=os.path.getmtime,
            reverse=True)
        for path in candidates:
            try:
                with open(path, newline='') as source:
                    names={row.get('pose') for row in csv.DictReader(source)}
            except (OSError, csv.Error):
                continue
            if names==MyFrame.PAYLOAD_CALIBRATION_POSE_NAMES:
                return path
        return None

    def request_payload_calibration(self, event=None, resume_samples=False):
        with self.payload_calibration_lock:
            process=self.payload_calibration_process
        if process is not None and process.poll() is None:
            self.show_local_result(False, '末端负载标定已经在运行。')
            return

        servo_enabled, faulted, servo_seen, fault_seen=self.get_cached_robot_state()
        with self.freedrive_state_lock:
            freedrive_state=self.freedrive_state
        if not servo_seen or not fault_seen or not servo_enabled or faulted:
            self.show_local_result(
                False,
                '自动标定要求真机 Servo On、无 Fault，并已收到最新驱动状态。')
            return
        if freedrive_state not in ('READY',):
            self.show_local_result(
                False,
                '自动标定只能从 READY 的位置保持状态开始；当前为 '+freedrive_state+'。')
            return

        try:
            self.listener.waitForTransform(
                'elfin_base', 'elfin_end_link', rospy.Time(0),
                rospy.Duration(0.5))
            flange_xyz, unused_quaternion=self.listener.lookupTransform(
                'elfin_base', 'elfin_end_link', rospy.Time(0))
            del unused_quaternion
            flange_height=float(flange_xyz[2])
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException, rospy.ROSException) as error:
            self.show_local_result(
                False, '无法核对标定起点的法兰高度：'+str(error))
            return
        if (not resume_samples and
                flange_height < self.PAYLOAD_CALIBRATION_MINIMUM_FLANGE_HEIGHT):
            self.show_local_result(
                False,
                '当前法兰高度 {:.3f} m，低于自动标定门限 {:.3f} m。'
                '请先用普通位置点动把末端移到上半工作区；完整标定不会在未知末端外形下从低位自行抬升。'.format(
                    flange_height,
                    self.PAYLOAD_CALIBRATION_MINIMUM_FLANGE_HEIGHT))
            return

        sample_path=None
        if resume_samples:
            sample_path=self.latest_payload_sample_path()
            if sample_path is None:
                self.show_local_result(False, '没有找到与当前 7 姿态轨迹兼容的完整 static_pairs.csv。')
                return
            message=(
                '本次不会重走当前 7 姿态轨迹，只会重新计算下列完整样本，并在实际工作姿态做一次不超过 1 秒的高阻尼 FREE 保持：\n\n'
                '{}\n\n'
                '继续即确认：\n'
                '1. 当前末端、转接件和线缆与该样本采集时完全相同，期间没有拆装、移动或增减物体；\n'
                '2. 当前机械臂静止、法兰高度 {:.3f} m；若低于 0.45 m，程序会先以 5% 经 MoveIt 抬到实际工作姿态，轨迹不得比当前高度再下降超过 15 mm；\n'
                '3. 抬升及保持区域、夹点已经清空，专人守上游电闸，所有人均离开连杆和末端可能移动的区域；\n'
                '4. 每段仍检查 MoveIt、全臂 z=0 与当前 STEP 包络 z=0.30 m 门禁；任一不符都不会进入 FREE。\n\n'
                '末端或线缆有任何变化，请选择“否”并重新执行完整标定。').format(
                    sample_path, flange_height)
            dialog_title='确认复用负载样本并短时验证'
        else:
            message=(
                '标定会自主移动真实机械臂，完成 5 个拟合和 2 个独立留出姿态的双向采样，约 29 段 5% 低速位置运动，随后进行一次不超过 1 秒的零力保持检验。\n\n'
                '继续即确认：\n'
                '1. 当前夹爪、剪刀、相机、转接件和线缆均已牢固固定，总质量不超过 E05 额定 5 kg；\n'
                '2. 机械臂全扫掠区和夹点已经清空，当前 STEP 包络外没有未建模障碍物或悬垂线缆；\n'
                '3. 所有人均在扫掠区外，专人守上游电闸，机械臂没有承载人员或危险物；\n'
                '4. 当前法兰高度为 {:.3f} m；脚本要求法兰不低于 0.45 m、活动连杆不穿 z=0、STEP 包络角点不低于 z=0.30 m；\n'
                '5. 任一 MoveIt、驱动状态、辨识残差或短时保持门禁失败都会停止并回滚。\n\n'
                '任一条件不满足请选择“否”。').format(flange_height)
            dialog_title='确认开始未知末端自动标定'
        confirm=wx.MessageDialog(
            self.advanced_dlg, message, dialog_title,
            wx.YES_NO | wx.NO_DEFAULT | wx.ICON_WARNING)
        accepted=(confirm.ShowModal()==wx.ID_YES)
        confirm.Destroy()
        if not accepted:
            self.append_event('负载标定', '操作者取消了清场确认。')
            return

        command=[
            '/opt/ros/noetic/bin/rosrun',
            'elfin_freedrive_controller',
            'calibrate_elfin_payload.py']
        if resume_samples:
            command.extend(['--resume-samples', sample_path])
        else:
            command.append('--execute')
        command.append('--confirmed-by-panel')
        try:
            process=subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                preexec_fn=os.setsid)
        except (OSError, subprocess.SubprocessError) as error:
            self.show_local_result(False, '无法启动自动负载标定：'+str(error))
            return
        with self.payload_calibration_lock:
            self.payload_calibration_process=process
        self.payload_calibration_output.SetValue(
            ('已确认清场，正在复用样本并启动短时保持验证...\n'
             if resume_samples else
             '已确认清场，正在启动自动负载标定...\n'))
        self.payload_calibration_start_btn.Disable()
        self.payload_calibration_resume_btn.Disable()
        self.payload_calibration_cancel_btn.Enable(True)
        self.apply_freedrive_state_to_controls(freedrive_state)
        self.append_event(
            '负载标定',
            ('已确认清场，复用 '+sample_path+' 并启动最多 1 秒保持验证。'
             if resume_samples else
             '已确认清场，启动高位双向静态辨识。'),
            'WARN')
        worker=threading.Thread(
            target=self.payload_calibration_worker, args=(process,))
        worker.daemon=True
        worker.start()
        if event is not None:
            event.Skip()

    def payload_calibration_worker(self, process):
        try:
            for line in iter(process.stdout.readline, ''):
                if not line:
                    break
                wx.CallAfter(self.append_payload_calibration_line, line.rstrip())
            return_code=process.wait()
        except Exception as error:
            return_code=-1
            wx.CallAfter(
                self.append_payload_calibration_line,
                '[失败] Panel 读取标定进程异常：'+str(error))
        finally:
            try:
                process.stdout.close()
            except Exception:
                pass
        wx.CallAfter(self.finish_payload_calibration, process, return_code)

    def append_payload_calibration_line(self, line):
        line=re.sub(
            r'\x1b\[[0-?]*[ -/]*[@-~]', '', str(line)).strip()
        if not line:
            return
        selection=self.payload_calibration_output.GetSelection()
        scroll=self.payload_calibration_output.GetScrollPos(wx.VERTICAL)
        self.payload_calibration_output.AppendText(line+'\n')
        if (selection[0] != selection[1] or
                self.payload_calibration_output.HasFocus()):
            end=self.payload_calibration_output.GetLastPosition()
            self.payload_calibration_output.SetSelection(
                min(selection[0], end), min(selection[1], end))
            self.payload_calibration_output.SetScrollPos(wx.VERTICAL, scroll)
        else:
            self.payload_calibration_output.SetInsertionPointEnd()
            self.payload_calibration_output.ShowPosition(
                self.payload_calibration_output.GetLastPosition())
        level='ERROR' if line.startswith('[失败]') else (
            'WARN' if line.startswith(('[回滚]', '[保持验证]')) else 'INFO')
        self.append_event('负载标定', line, level)

    def finish_payload_calibration(self, process, return_code):
        with self.payload_calibration_lock:
            if self.payload_calibration_process is process:
                self.payload_calibration_process=None
        with self.freedrive_state_lock:
            freedrive_state=self.freedrive_state
        self.apply_freedrive_state_to_controls(freedrive_state)
        if return_code == 0:
            self.append_event(
                '负载标定', '自动标定进程正常结束；当前配置已由管理器回报。')
        else:
            self.append_event(
                '负载标定',
                '自动标定未完成（退出码 '+str(return_code)+
                '）；请查看上方最后一条“失败/回滚”原因。',
                'ERROR')

    def cancel_payload_calibration(self, event=None):
        with self.payload_calibration_lock:
            process=self.payload_calibration_process
        if process is None or process.poll() is not None:
            self.payload_calibration_cancel_btn.Disable()
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            self.payload_calibration_cancel_btn.Disable()
            self.append_event(
                '负载标定',
                '已请求中止；等待脚本停止轨迹、退出 FREE、回滚候选并恢复阻尼。',
                'WARN')
        except OSError as error:
            self.append_event(
                '负载标定', '发送中止请求失败：'+str(error), 'ERROR')
        if event is not None:
            event.Skip()

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
        self.append_event(
            '动作', '按住 J'+str(abs(mark))+
            (' 正向' if mark > 0 else ' 负向')+'点动')
        self.call_teleop_joint_req.data=mark
        resp=self.call_teleop_joint.call(self.call_teleop_joint_req)
        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()
        
    def teleop_pcs(self,event,mark): 
        axes=('X', 'Y', 'Z', 'Rx', 'Ry', 'Rz')
        axis=axes[abs(mark)-1] if 0 < abs(mark) <= len(axes) else str(mark)
        self.append_event(
            '动作', '按住 '+axis+
            (' 正向' if mark > 0 else ' 负向')+'点动')
        self.call_teleop_cart_req.data=mark            
        resp=self.call_teleop_cart.call(self.call_teleop_cart_req)
        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()    
    
    def release_button(self, event, mark):
        self.append_event('动作', '松开点动按键，发送 Stop')
        self.call_teleop_stop_req.data=True
        resp=self.call_teleop_stop.call(self.call_teleop_stop_req)
        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()
    
    def call_set_bool_common(self, event, client, request):
        btn=event.GetEventObject()
        self.append_event('动作', btn.GetName() or btn.GetLabel())
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
            self.append_event('动作', '先取消当前轨迹，再请求 Servo Off')
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
        """Keep the raw input word and report external input transitions."""
        raw=int(value) & 0xffff
        changed_inputs=[]
        with self.DI_show_lock:
            had_previous=self.di_seen
            previous_inputs=list(self.DI_show[:len(self.DI_display)])
            self.di_raw_value=raw
            self.di_seen=True
            for i in range(len(self.DI_show)):
                self.DI_show[i]=(raw >> i) & 0x01
            if had_previous:
                changed_inputs=[
                    (index, self.DI_show[index])
                    for index, previous in enumerate(previous_inputs)
                    if self.DI_show[index] != previous]
        with self.tool_button_lock:
            for name, bit in self.tool_button_bits.items():
                self.tool_button_pressed[name]=bool((raw >> bit) & 0x01)
        for index, state in changed_inputs:
            wx.CallAfter(self.log_input_state_change, index, bool(state))

    def log_input_state_change(self, index, state):
        """Append one result line when an external E05 input changes."""
        if index < 0 or index >= len(self.DI_display):
            return
        label='DI'+str(index)+' / INPUT_'+str(index)
        self.append_event(
            '结果',
            '完成：'+label+' 当前为 '+('ON' if state else 'OFF')+
            '（输入状态变化确认）')

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
        self.io_status_show.SetValue('I/O\n'+('在线' if online else '离线'))
        self.io_status_show.SetToolTip(text)
        self.io_status_show.SetBackgroundColour(
            wx.Colour(200, 235, 200) if online else wx.Colour(245, 205, 205))
        self.append_event(
            '状态', text, 'INFO' if online else 'WARN', dedup_key='io_state')

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
            write_request = ElfinIODWriteRequest()
            write_request.digital_output = int(request)
            response = client.call(write_request)
            if not response.success:
                result.success = False
                result.message = label + ' write was rejected by the driver'
            else:
                # The SDO write and its hardware-visible readback can straddle
                # EtherCAT cycles.  An immediate single read produced false
                # failures on the x86 host, even though the output changed on
                # the following cycle.  Poll for a bounded interval and never
                # issue a second write automatically.
                deadline = time.monotonic() + 0.75
                observed = current
                while True:
                    observed = self.call_read_do.call(
                        self.call_read_do_req).digital_input
                    if ((observed & output_mask) ==
                            (request & output_mask)):
                        break
                    if time.monotonic() >= deadline:
                        break
                    time.sleep(0.025)
                self.process_DO_btn(observed)
                result.success = ((observed & output_mask) == (request & output_mask))
                if result.success:
                    state = 'ON' if ((observed >> bit) & 0x01) else 'OFF'
                    result.message = label + ' 当前为 '+state+'（写后回读确认）'
                else:
                    result.message = (
                        label+' 在 0.75 秒内写后回读不一致；Panel 未重复写入')
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
                'OUTPUT_'+str(i)+'\n'+
                ('ON' if state else 'OFF'))
            self.DO_btn_display[i].SetBackgroundColour(
                wx.Colour(200, 235, 200) if state else wx.NullColour)
        for i, state in enumerate(input_states):
            self.DI_display[i].SetValue(
                'INPUT_'+str(i)+'\n'+('ON' if state else 'OFF'))
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
                    ('FREE 按键' if name == 'FREE' else 'POINT')+'\n等待 I/O')
                self.tool_button_state_show[name].SetBackgroundColour(wx.NullColour)
            else:
                state='按下' if pressed[name] else '释放'
                self.tool_button_state_show[name].SetValue(
                    ('FREE 按键' if name == 'FREE' else 'POINT')+'\n'+state)
                self.tool_button_state_show[name].SetBackgroundColour(
                    wx.Colour(200, 235, 200) if pressed[name] else wx.NullColour)

    def request_freedrive(self, event, enable):
        self.append_event(
            '动作', '请求'+('进入 FREE（持续到人工退出）'
                            if enable else '退出 FREE 并保持当前位置'))
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
        self.append_event('动作', '设置 FREE 速度保护上限为 '+
                          str(self.freedrive_speed_slider.GetValue())+'%')
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
        self.append_event(
            '动作', '设置六轴阻尼：'+', '.join(
                'J'+str(index+1)+'='+str(round(value*100))+'%'
                for index, value in enumerate(scales)))
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
        self.append_event('动作', '请求记录当前六轴姿态')
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
        if response.success and self.pose_manager_dlg.IsShown():
            self.request_pose_list()

    def request_pose_list(self, event=None):
        self.pose_refresh_btn.Disable()
        worker=threading.Thread(target=self.pose_list_worker)
        worker.daemon=True
        worker.start()
        if event is not None:
            event.Skip()

    def pose_list_worker(self):
        try:
            self.call_list_freedrive_points.wait_for_service(timeout=1.0)
            response=self.call_list_freedrive_points.call(
                ListRecordedPointsRequest())
            wx.CallAfter(self.finish_pose_list_request, response, '')
        except (rospy.ROSException, rospy.ServiceException) as error:
            wx.CallAfter(
                self.finish_pose_list_request, None,
                '姿态列表服务不可用：'+str(error))

    def finish_pose_list_request(self, response, error):
        self.pose_refresh_btn.Enable(True)
        self.pose_list.DeleteAllItems()
        self.pose_select_all_btn.Disable()
        self.pose_delete_btn.Disable()
        if response is None:
            self.append_event('姿态', error, 'ERROR')
            return
        self.pose_record_file_show.SetValue(response.record_file)
        self.pose_record_file_show.SetToolTip(response.record_file)
        if not response.success:
            self.append_event('姿态', response.message, 'ERROR')
            return
        count=len(response.indices)
        if (len(response.stamps) != count or
                len(response.sources) != count or
                len(response.joints_rad) != count*6):
            self.append_event(
                '姿态', '姿态列表服务返回数组长度不一致。', 'ERROR')
            return
        for row in range(count):
            index=response.indices[row]
            stamp=time.strftime(
                '%Y-%m-%d %H:%M:%S', time.localtime(response.stamps[row]))
            item=self.pose_list.InsertItem(row, str(index))
            self.pose_list.SetItem(item, 1, stamp)
            self.pose_list.SetItem(item, 2, response.sources[row])
            for joint in range(6):
                degrees=response.joints_rad[row*6+joint]*180.0/math.pi
                self.pose_list.SetItem(item, 3+joint, '{:.3f}'.format(degrees))
        self.pose_select_all_btn.Enable(count > 0)
        self.update_pose_selection_controls()
        self.append_event(
            '姿态', response.message+'；文件 '+response.record_file,
            dedup_key='point_list')

    def selected_pose_indices(self):
        indices=[]
        selected=self.pose_list.GetFirstSelected()
        while selected >= 0:
            indices.append(int(self.pose_list.GetItemText(selected, 0)))
            selected=self.pose_list.GetNextSelected(selected)
        return indices

    def update_pose_selection_controls(self, event=None):
        has_selection=self.pose_list.GetFirstSelected() >= 0
        self.pose_delete_btn.Enable(
            has_selection and not self.pose_delete_in_progress)
        if event is not None:
            event.Skip()

    def select_all_poses(self, event=None):
        self.pose_list.Freeze()
        try:
            for row in range(self.pose_list.GetItemCount()):
                self.pose_list.SetItemState(
                    row, wx.LIST_STATE_SELECTED, wx.LIST_STATE_SELECTED)
        finally:
            self.pose_list.Thaw()
        self.update_pose_selection_controls()
        if event is not None:
            event.Skip()

    def request_delete_pose(self, event=None):
        indices=self.selected_pose_indices()
        if not indices:
            self.append_event('姿态', '请先选中至少一条姿态。', 'WARN')
            return
        delete_all=(len(indices) == self.pose_list.GetItemCount())
        if delete_all:
            detail='全部 '+str(len(indices))+' 条姿态'
            request_indices=[0]
        else:
            sorted_indices=sorted(indices)
            shown_indices=sorted_indices[:12]
            index_text=', '.join(str(index) for index in shown_indices)
            if len(sorted_indices) > len(shown_indices):
                index_text+=', ...'
            detail=(str(len(indices))+' 条选中姿态（序号 '+index_text+'）')
            request_indices=sorted(indices, reverse=True)
        confirm=wx.MessageDialog(
            self.pose_manager_dlg,
            '确定删除'+detail+'吗？\n\n文件会原子替换；保留姿态的序号将连续重排。',
            '确认删除 POINT 姿态',
            wx.YES_NO | wx.NO_DEFAULT | wx.ICON_WARNING)
        accepted=(confirm.ShowModal()==wx.ID_YES)
        confirm.Destroy()
        if not accepted:
            return
        self.pose_delete_in_progress=True
        self.pose_delete_btn.Disable()
        self.pose_select_all_btn.Disable()
        self.pose_refresh_btn.Disable()
        self.record_point_btn.Disable()
        worker=threading.Thread(
            target=self.delete_pose_worker, args=(request_indices, delete_all))
        worker.daemon=True
        worker.start()
        if event is not None:
            event.Skip()

    def delete_pose_worker(self, indices, delete_all):
        deleted_count=0
        try:
            self.call_delete_freedrive_point.wait_for_service(timeout=1.0)
            for index in indices:
                response=self.call_delete_freedrive_point.call(
                    DeleteRecordedPointRequest(index=index))
                if not response.success:
                    if deleted_count:
                        response.message=(
                            '已删除 '+str(deleted_count)+' 条，随后失败：'+
                            response.message)
                    wx.CallAfter(self.finish_delete_pose_request, response)
                    return
                deleted_count+=1
            if delete_all or len(indices) == 1:
                final_response=response
            else:
                final_response=SetBoolResponse(
                    success=True,
                    message='已删除 '+str(deleted_count)+' 条选中姿态')
            wx.CallAfter(self.finish_delete_pose_request, final_response)
        except (rospy.ROSException, rospy.ServiceException) as error:
            prefix=('已删除 '+str(deleted_count)+' 条，随后' if deleted_count
                    else '')
            response=SetBoolResponse(
                success=False,
                message=prefix+'姿态删除服务不可用：'+str(error))
            wx.CallAfter(self.finish_delete_pose_request, response)

    def finish_delete_pose_request(self, response):
        self.pose_delete_in_progress=False
        self.record_point_btn.Enable(True)
        self.update_reply_show(response)
        self.request_pose_list()

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
            style=wx.TE_MULTILINE | wx.TE_READONLY |
                  getattr(wx, 'TE_CHARWRAP', wx.TE_WORDWRAP),
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
        with self.payload_calibration_lock:
            calibration_process=self.payload_calibration_process
        if (calibration_process is not None
                and calibration_process.poll() is None
                and event.CanVeto()):
            self.cancel_payload_calibration()
            self.show_local_result(
                False,
                'Panel 暂停退出：自动负载标定正在停止并恢复控制器，请等待日志显示进程结束。')
            event.Veto()
            return
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
        self.append_event(
            '维护', message,
            'INFO' if success is not False else 'ERROR')

    def get_cached_robot_state(self):
        now=time.monotonic()
        with self.servo_state_lock:
            servo_enabled=self.servo_state
            servo_state_received=(
                self.servo_state_received and
                now-self.servo_state_received_at < self.DRIVER_STATE_TIMEOUT)
        with self.fault_state_lock:
            faulted=self.fault_state
            fault_state_received=(
                self.fault_state_received and
                now-self.fault_state_received_at < self.DRIVER_STATE_TIMEOUT)
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
        self.joint_state_show.SetValue('关节反馈\n在线')
        self.joint_state_show.SetBackgroundColour(wx.Colour(200, 235, 200))
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
        message=str(msg.message).strip()
        summary=('完成：' if msg.success else '失败：')+message
        self.append_event(
            '结果', summary, 'INFO' if msg.success else 'ERROR')
            
    def update_servo_state(self, msg):
        if msg.data:
            self.servo_state_show.SetBackgroundColour(wx.Colour(200, 225, 200))
            self.servo_state_show.SetValue('SERVO\nON')
        else:
            self.servo_state_show.SetBackgroundColour(wx.Colour(225, 200, 200))
            self.servo_state_show.SetValue('SERVO\nOFF')
        self.append_event(
            '状态', '机器人已 Servo On' if msg.data else '机器人已 Servo Off',
            dedup_key='servo_state')
    
    def update_fault_state(self, msg):
        if msg.data:
            self.fault_state_show.SetBackgroundColour(wx.Colour(225, 200, 200))
            self.fault_state_show.SetValue('FAULT\n已触发')
        else:
            self.fault_state_show.SetBackgroundColour(wx.Colour(200, 225, 200))
            self.fault_state_show.SetValue('FAULT\n正常')
        self.append_event(
            '状态', '底层 Fault 已触发' if msg.data else '当前无 Fault',
            'ERROR' if msg.data else 'INFO', dedup_key='fault_state')

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
        indicator_labels={
            'STARTING': '启动中',
            'WAITING': '等待',
            'LOCKED': '入口锁定',
            'SERVO_OFF': 'Servo Off',
            'FAULT': 'Fault',
            'READY': '可进入',
            'ENTERING': '切入中',
            'ACTIVE': '拖拽中',
            'EXITING': '退出中',
            'RECOVERING': '恢复中',
            'HOLDING': '位置保持',
            'FALLBACK': '保护回退',
            'ERROR': '回退失败'}
        self.freedrive_state_show.SetValue(
            'FREE\n'+indicator_labels.get(state, state))
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
        level='ERROR' if state in ('ERROR', 'FAULT', 'FALLBACK') else 'INFO'
        self.append_event(
            '状态', 'FREE '+labels.get(state, state), level,
            dedup_key='freedrive_state')
        self.apply_freedrive_state_to_controls(state)

    def apply_freedrive_state_to_controls(self, state):
        with self.payload_calibration_lock:
            calibration_process=self.payload_calibration_process
        calibrating=(calibration_process is not None
                     and calibration_process.poll() is None)
        owns_joints=(state in (
            'ENTERING', 'ACTIVE', 'EXITING', 'RECOVERING', 'HOLDING', 'FALLBACK')
            or calibrating)
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
        self.payload_calibration_start_btn.Enable(not owns_joints)
        self.payload_calibration_resume_btn.Enable(not owns_joints)
        self.payload_calibration_cancel_btn.Enable(calibrating)
        # These remain available as independent stop paths during a bad switch.
        self.power_off_btn.Enable(True)
        self.stop_btn.Enable(True)

    def update_freedrive_detail(self, detail):
        self.freedrive_detail_show.SetValue(detail)
        self.freedrive_detail_show.SetToolTip(detail)
        level=('ERROR' if any(word in detail for word in ('失败', 'Fault', '过期', '丢失'))
               else 'WARN' if any(word in detail for word in ('警告', '保护', '拒绝'))
               else 'INFO')
        self.append_event(
            'FREE 原因', detail, level, dedup_key='freedrive_detail')

    def update_freedrive_ring(self, state):
        mapping={
            'RED_FAULT_EXPECTED': ('红 / Fault', wx.Colour(245, 205, 205)),
            'BLUE_ZERO_FORCE_EXPECTED': ('蓝 / FREE', wx.Colour(190, 220, 250)),
            'YELLOW_SERVO_OFF_EXPECTED': ('黄 / Servo Off', wx.Colour(245, 235, 180)),
            'GREEN_SERVO_ON_EXPECTED': ('绿 / Servo On', wx.Colour(200, 235, 200)),
            'UNKNOWN_RING_STATE': ('未知', wx.NullColour)}
        label, colour=mapping.get(state, (state, wx.NullColour))
        self.freedrive_ring_show.SetValue('灯环\n'+label)
        self.freedrive_ring_show.SetBackgroundColour(colour)

    def update_freedrive_point_count(self, count):
        self.freedrive_point_count_show.SetValue('姿态记录\n'+str(count)+' 条')
        self.freedrive_point_count_show.SetToolTip(
            '已持久记录 '+str(count)+' 个 POINT 姿态；点击“姿态管理”查看')

    def update_freedrive_validation(self, detail):
        self.freedrive_validation_show.SetValue(detail)
        if detail.startswith('通过'):
            result='通过'
            colour=wx.Colour(200, 235, 200)
        elif detail.startswith('警告'):
            result='警告'
            colour=wx.Colour(245, 235, 180)
        elif detail.startswith('未通过'):
            result='未通过'
            colour=wx.Colour(245, 205, 205)
        else:
            result='等待'
            colour=wx.NullColour
        self.freedrive_validation_show.SetBackgroundColour(colour)
        values={'result': result}
        patterns={
            'excited': r'有效轴=([^，；]+)',
            'reverse': r'反向轴=([^，；]+)',
            'alignment': r'方向一致度=([^，；]+)',
            'scale': r'反馈/模型比例=([^，；]+)',
            'residual': r'归一化残差=([^，；]+)',
            'stddev': r'最大力矩波动=([^ ]+)\s*Nm',
            'capacity': r'重力容量=([^（，；]+)'}
        for key, pattern in patterns.items():
            match=re.search(pattern, detail)
            values[key]=match.group(1) if match else '--'
        for key, control in self.validation_metric_shows.items():
            control.SetValue(
                self.validation_metric_labels[key]+'\n'+values.get(key, '--'))
            control.SetToolTip(detail)
            control.SetBackgroundColour(colour if key == 'result' else wx.NullColour)
        hold_verified='实际保持验证已通过' in detail
        event_class='hold_verified' if hold_verified else result
        previous=self.event_log_last.get('_validation_class')
        if previous != event_class:
            self.event_log_last['_validation_class']=event_class
            level=('ERROR' if result == '未通过'
                   else 'WARN' if result == '警告' else 'INFO')
            self.append_event('重力预检', detail, level)

    def update_freedrive_trial_log(self, path):
        self.freedrive_trial_log_show.SetValue(path or '尚未开始本次试验')
        self.freedrive_trial_log_show.SetToolTip(
            path or '进入 FREE 后管理器会创建逐周期 CSV')
        if path:
            self.append_event(
                '数据', '本次 FREE CSV：'+path,
                dedup_key='freedrive_trial_log')

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
        self.freedrive_speed_slider.SetToolTip(
            '拖拽速度保护倍率 50% 到 300%；退出 FREE 后应用。\n'+text)

    def freedrive_velocity_scale_cb(self, data):
        with self.freedrive_state_lock:
            self.freedrive_velocity_scale=float(data.data)
        value=max(50, min(300, int(round(float(data.data)*100))))
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

    def payload_profile_cb(self, data):
        self.payload_profile=data.data
        wx.CallAfter(self.update_payload_profile, data.data)

    def update_payload_profile(self, profile):
        self.payload_profile_show.SetValue(profile)
        self.payload_profile_show.SetToolTip(profile)
        self.append_event(
            '负载配置', profile, dedup_key='payload_profile')

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
        wx.CallAfter(self.handle_recorded_point_event)

    def handle_recorded_point_event(self):
        if self.recorded_point_seen:
            message='新的 POINT 姿态已记录；完整六轴数值见“姿态管理”。'
        else:
            message='已载入最近一次 POINT 记录；完整六轴数值见“姿态管理”。'
            self.recorded_point_seen=True
        self.append_event('姿态', message)
        if self.pose_manager_dlg.IsShown():
            self.request_pose_list()

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
            self.servo_state_received_at=time.monotonic()
            self.servo_state_lock.release()
        wx.CallAfter(self.update_servo_state, data)
    
    def fault_state_cb(self, data):
        if self.fault_state_lock.acquire():
            self.fault_state=data.data
            self.fault_state_received=True
            self.fault_state_received_at=time.monotonic()
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
        rospy.Subscriber('/elfin_freedrive_manager/payload_profile', String,
                         self.payload_profile_cb)

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
