import sys
import argparse
from PyQt5 import QtWidgets
from configparser import ConfigParser
from thread_training import TrainingThread
from train_ui import TrainingUi


def main():
    config_file = 'D:/Unreal/Unreal Projects/RL/configs/SimpleMultirotor.ini'

    app = QtWidgets.QApplication(sys.argv)
    gui = TrainingUi(config_file)
    gui.show()

    training_thread = TrainingThread(config_file)

    training_thread.env.action_signal.connect(gui.action_cb)
    training_thread.env.state_signal.connect(gui.state_cb)
    training_thread.env.attitude_signal.connect(gui.attitude_plot_cb)
    training_thread.env.reward_signal.connect(gui.reward_plot_cb)

    cfg = ConfigParser()
    cfg.read(config_file)

    training_thread.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print('system exit')
