import sys
from PyQt5 import QtWidgets
from thread_evaluation import EvaluateThread
from configparser import ConfigParser
from train_ui import TrainingUi

def main():
    eval_path = r'D:\Unreal\Unreal Projects\RL\scripts\logs\Forest\2025_04_15_11_24_SimpleMultirotor_mlp_SAC'
    config_file = eval_path + '/config/config.ini'
    model_file = eval_path + '/models/model.zip'
    total_eval_episodes = 50

    app = QtWidgets.QApplication(sys.argv)
    gui = TrainingUi(config=config_file)
    gui.show()

    evaluate_thread = EvaluateThread(eval_path, config_file, model_file, total_eval_episodes)
    evaluate_thread.env.action_signal.connect(gui.action_cb)
    evaluate_thread.env.state_signal.connect(gui.state_cb)
    evaluate_thread.env.attitude_signal.connect(gui.attitude_plot_cb)
    evaluate_thread.env.reward_signal.connect(gui.reward_plot_cb)

    cfg = ConfigParser()
    cfg.read(config_file)

    evaluate_thread.start()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
