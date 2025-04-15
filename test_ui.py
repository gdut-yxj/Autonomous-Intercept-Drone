import sys
import math
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtWidgets
from PIL import Image
from PyQt5.QtWidgets import QGroupBox, QHBoxLayout, QVBoxLayout, QWidget
from pyqtgraph.widgets.MatplotlibWidget import MatplotlibWidget
from configparser import ConfigParser


class TrainingUi(QWidget):
    def __init__(self, config):
        super(TrainingUi, self).__init__()
        self.cfg = ConfigParser()
        self.cfg.read(config)
        self.max_len = 0
        self.ptr = 0
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Training UI")
        pg.setConfigOptions(leftButtonPan=False)
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        pg.setConfigOption('imageAxisOrder', 'row-major')
        self.max_len = 100
        self.ptr = -self.max_len
        self.dynamics = self.cfg.get('options', 'dynamic_name')
        action_plot_gb = self.create_actionPlot_groupBox_multirotor()
        state_plot_gb = self.create_state_plot_groupbox()
        attitude_plot_gb = self.create_attitude_plot_groupbox()
        reward_plot_gb = self.create_reward_plot_groupbox()

        right_widget = QWidget()
        vlayout = QVBoxLayout()
        vlayout.addWidget(reward_plot_gb)
        right_widget.setLayout(vlayout)

        main_layout = QHBoxLayout()
        main_layout.addWidget(action_plot_gb)
        main_layout.addWidget(state_plot_gb)
        main_layout.addWidget(attitude_plot_gb)
        main_layout.addWidget(right_widget)

        self.setLayout(main_layout)
        self.pen_red = pg.mkPen(color='r', width=2)
        self.pen_blue = pg.mkPen(color='b', width=1)
        self.pen_green = pg.mkPen(color='g', width=2)

    def update_value_list(self, list, value):
        list[:-1] = list[1:]
        list[-1] = float(value)
        return list

    def create_actionPlot_groupBox_multirotor(self):
        actionPlotGroupBox = QGroupBox('Action (multirotor)')
        self.v_xy_cmd_list = np.linspace(0, 0, self.max_len)
        self.v_xy_real_list = np.linspace(0, 0, self.max_len)
        self.v_z_cmd_list = np.linspace(0, 0, self.max_len)
        self.v_z_real_list = np.linspace(0, 0, self.max_len)
        self.yaw_rate_cmd_list = np.linspace(0, 0, self.max_len)
        self.yaw_rate_list = np.linspace(0, 0, self.max_len)
        layout = QVBoxLayout()

        self.plotWidget_v_xy = pg.PlotWidget(title='v_xy (m/s)')
        self.plotWidget_v_xy.setYRange(max=self.cfg.getfloat(
            'multirotor', 'v_xy_max'), min=self.cfg.getfloat('multirotor', 'v_xy_min'))
        self.plotWidget_v_xy.showGrid(x=True, y=True)
        self.plot_v_xy = self.plotWidget_v_xy.plot()
        self.plot_v_xy_cmd = self.plotWidget_v_xy.plot()

        self.plotWidget_v_z = pg.PlotWidget(title='v_z (m/s)')
        self.plotWidget_v_z.setYRange(max=self.cfg.getfloat(
            'multirotor', 'v_z_max'), min=-self.cfg.getfloat('multirotor', 'v_z_max'))
        self.plotWidget_v_z.showGrid(x=True, y=True)
        self.plot_v_z = self.plotWidget_v_z.plot()
        self.plot_v_z_cmd = self.plotWidget_v_z.plot()

        self.plotWidget_yaw_rate = pg.PlotWidget(title='yaw_rate (deg/s)')
        self.plotWidget_yaw_rate.setYRange(max=self.cfg.getfloat(
            'multirotor', 'yaw_rate_max_deg'), min=-self.cfg.getfloat('multirotor', 'yaw_rate_max_deg'))
        self.plotWidget_yaw_rate.showGrid(x=True, y=True)
        self.plot_yaw_rate = self.plotWidget_yaw_rate.plot()
        self.plot_yaw_rate_cmd = self.plotWidget_yaw_rate.plot()

        layout.addWidget(self.plotWidget_v_xy)
        layout.addWidget(self.plotWidget_v_z)
        layout.addWidget(self.plotWidget_yaw_rate)

        actionPlotGroupBox.setLayout(layout)

        return actionPlotGroupBox

    def action_cb(self,step, action):
        self.action_cb_multirotor(step, action)

    def action_cb_multirotor(self, step, action):
        self.update_value_list(self.v_xy_cmd_list, action[0])
        self.update_value_list(self.v_z_cmd_list, action[1])
        self.update_value_list(self.yaw_rate_cmd_list, math.degrees(action[2]))

        self.plot_v_xy_cmd.setData(self.v_xy_cmd_list, pen=self.pen_blue)
        self.plot_v_z_cmd.setData(self.v_z_cmd_list, pen=self.pen_blue)
        self.plot_yaw_rate_cmd.setData(self.yaw_rate_cmd_list, pen=self.pen_blue)

    def create_state_plot_groupbox(self):
        state_plot_groupbox = QGroupBox(title='State feature')
        self.distance_list = np.linspace(0, 0, self.max_len)
        self.vertical_dis_list = np.linspace(0, 0, self.max_len)
        self.relative_yaw_list = np.linspace(0, 0, self.max_len)

        layout = QVBoxLayout()

        self.pw1 = pg.PlotWidget(title='distance_xy (m)')
        self.pw1.showGrid(x=True, y=True)
        self.p1 = self.pw1.plot()

        self.pw2 = pg.PlotWidget(title='distance_z (m)')
        self.pw2.showGrid(x=True, y=True)
        self.p2 = self.pw2.plot()

        self.pw3 = pg.PlotWidget(title='relative yaw (deg)')
        self.pw3.showGrid(x=True, y=True)
        self.p3 = self.pw3.plot()

        layout.addWidget(self.pw1)
        layout.addWidget(self.pw2)
        layout.addWidget(self.pw3)

        state_plot_groupbox.setLayout(layout)
        return state_plot_groupbox

    def state_cb(self, step, state_raw):
        self.update_value_list(self.distance_list, state_raw[0])
        self.update_value_list(self.vertical_dis_list, state_raw[1])
        self.update_value_list(self.relative_yaw_list, state_raw[2])

        self.p1.setData(self.distance_list, pen=self.pen_red)
        self.p2.setData(self.vertical_dis_list, pen=self.pen_red)
        self.p3.setData(self.relative_yaw_list, pen=self.pen_red)

        # update action real
        self.update_value_list(self.v_xy_real_list, state_raw[3])
        self.update_value_list(self.v_z_real_list, state_raw[4])
        self.plot_v_xy.setData(self.v_xy_real_list, pen=self.pen_red)
        self.plot_v_z.setData(self.v_z_real_list, pen=self.pen_red)

        self.update_value_list(self.yaw_rate_list, state_raw[5])
        self.plot_yaw_rate.setData(self.yaw_rate_list, pen=self.pen_red)

    def create_attitude_plot_groupbox(self):
        plot_gb = QGroupBox(title='Attitude')
        layout = QVBoxLayout()

        self.roll_list = np.linspace(0, 0, self.max_len)
        self.roll_cmd_list = np.linspace(0, 0, self.max_len)

        self.pitch_list = np.linspace(0, 0, self.max_len)
        self.pitch_cmd_list = np.linspace(0, 0, self.max_len)

        self.yaw_list = np.linspace(0, 0, self.max_len)
        self.yaw_cmd_list = np.linspace(0, 0, self.max_len)

        self.pw_roll = pg.PlotWidget(title='roll (deg)')
        self.pw_roll.setYRange(max=45, min=-45)
        self.pw_roll.showGrid(x=True, y=True)
        self.plot_roll = self.pw_roll.plot()
        self.plot_roll_cmd = self.pw_roll.plot()

        self.pw_pitch = pg.PlotWidget(title='pitch (deg)')
        self.pw_pitch.setYRange(max=25, min=-25)
        self.pw_pitch.showGrid(x=True, y=True)
        self.plot_pitch = self.pw_pitch.plot()
        self.plot_pitch_cmd = self.pw_pitch.plot()

        self.pw_yaw = pg.PlotWidget(title='yaw (deg)')
        # self.pw_yaw.setYRange(max=180, min=-180)
        self.pw_yaw.showGrid(x=True, y=True)
        self.plot_yaw = self.pw_yaw.plot()
        self.plot_yaw_cmd = self.pw_yaw.plot()

        layout.addWidget(self.pw_roll)
        layout.addWidget(self.pw_pitch)
        layout.addWidget(self.pw_yaw)

        plot_gb.setLayout(layout)
        return plot_gb

    def attitude_plot_cb(self, step, attitude, attitude_cmd):
        self.update_value_list(self.pitch_list, math.degrees(attitude[0]))
        self.update_value_list(self.roll_list, math.degrees(attitude[1]))
        self.update_value_list(self.yaw_list, math.degrees(attitude[2]))

        self.plot_pitch.setData(self.pitch_list, pen=self.pen_red)
        self.plot_roll.setData(self.roll_list, pen=self.pen_red)
        self.plot_yaw.setData(self.yaw_list, pen=self.pen_red)

    def create_reward_plot_groupbox(self):
        reward_plot_groupbox = QGroupBox(title='Reward')
        layout = QHBoxLayout()
        # reward_plot_groupbox.setFixedHeight(300)
        reward_plot_groupbox.setFixedWidth(600)

        self.reward_list = np.linspace(0, 0, self.max_len)
        self.total_reward_list = np.linspace(0, 0, self.max_len)

        self.rw_pw_1 = pg.PlotWidget(title='reward')
        self.rw_pw_1.showGrid(x=True, y=True)
        self.rw_p_1 = self.rw_pw_1.plot()

        self.rw_pw_2 = pg.PlotWidget(title='total reward')
        self.rw_pw_2.showGrid(x=True, y=True)
        self.rw_p_2 = self.rw_pw_2.plot()

        layout.addWidget(self.rw_pw_1)
        layout.addWidget(self.rw_pw_2)

        reward_plot_groupbox.setLayout(layout)
        return reward_plot_groupbox

    def reward_plot_cb(self, step, reward, total_reward):
        self.update_value_list(self.reward_list, reward)
        self.update_value_list(self.total_reward_list, total_reward)

        self.rw_p_1.setData(self.reward_list, pen=self.pen_red)
        self.rw_p_2.setData(self.total_reward_list, pen=self.pen_red)


def main():
    config_file = 'configs/SimpleMultirotor.ini'
    app = QtWidgets.QApplication(sys.argv)
    gui = TrainingUi(config_file)
    gui.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
