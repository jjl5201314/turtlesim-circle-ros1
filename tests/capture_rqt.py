#!/usr/bin/env python3
"""Capture real Noetic rqt widgets against a live ROS graph in CI.

This small Qt host is for reproducible evidence on a headless runner. Normal
desktop use should start rqt through launch/visualize.launch instead. It uses
Noetic plugin internals and intentionally stays outside the installed package.
"""
import json
from pathlib import Path
import sys
import time

import rosgraph
import rospy
from geometry_msgs.msg import Twist
from python_qt_binding.QtCore import QObject
from python_qt_binding.QtWidgets import QApplication


class Context(QObject):
    def __init__(self, topics):
        super().__init__()
        self.topics = topics
        self.widget = None

    def argv(self):
        return self.topics

    def serial_number(self):
        return 1

    def add_widget(self, widget):
        self.widget = widget
        widget.resize(1500, 900)
        widget.show()


class Settings:
    def value(self, key, default=None):
        return default


def main():
    kind, output_arg = sys.argv[1:]
    assert kind in ('plot', 'graph')
    output = Path(output_arg)
    output.mkdir(parents=True, exist_ok=True)
    rospy.init_node('circle_' + kind, disable_signals=True)
    rospy.wait_for_message('/turtle1/cmd_vel', Twist, timeout=25)
    app = QApplication(['rqt_' + kind])
    topics = ['/turtle1/cmd_vel/linear/x', '/turtle1/cmd_vel/angular/z']
    context = Context(topics if kind == 'plot' else [])
    if kind == 'plot':
        from rqt_plot.plot import Plot
        plugin = Plot(context)
    else:
        from rqt_graph.ros_graph import RosGraph
        plugin = RosGraph(context)
    plugin.restore_settings(Settings(), Settings())
    if kind == 'graph':
        widget = context.widget
        widget.graph_type_combo_box.setCurrentIndex(2)
        widget.filter_line_edit.setText('/')
        widget.topic_filter_line_edit.setText('/')
        for name in ('dead_sinks_check_box', 'leaf_topics_check_box',
                     'quiet_check_box', 'unreachable_check_box',
                     'hide_tf_nodes_check_box', 'hide_dynamic_reconfigure_check_box'):
            getattr(widget, name).setChecked(False)
        widget.auto_fit_graph_check_box.setChecked(True)
    try:
        deadline = time.monotonic() + (13 if kind == 'plot' else 8)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            app.processEvents()
            time.sleep(0.02)
        if kind == 'graph':
            plugin._update_rosgraph()
            plugin._fit_in_view()
            dot = plugin._current_dotcode
            for label in ('/circle_controller', '/circle_recorder', '/circle_plot',
                          '/turtlesim', '/turtle1/cmd_vel', '/turtle1/pose'):
                assert label in dot, 'Missing ROS graph label: ' + label
            (output / 'rqt_graph.dot').write_text(dot, encoding='utf-8')
        else:
            assert set(topics).issubset(plugin._widget._rosdata), 'Missing plotted topics'
            # Keep both constant curves clear of the axes and include startup
            # in the screenshot; the backend restores a short scrolling view.
            plugin._widget.autoscroll_checkbox.setChecked(False)
            plugin._data_plot.autoscroll(False)
            plugin._data_plot.set_autoscale(x=False, y=False)
            plugin._data_plot.set_xlim([0, 15])
            plugin._data_plot.set_ylim([-0.1, 1.2])
            plugin._data_plot.redraw()
        for _ in range(10):
            app.processEvents()
            time.sleep(0.03)
        state = rosgraph.Master(rospy.get_name()).getSystemState()
        subscribers = dict(state[1])
        assert '/circle_plot' in subscribers['/turtle1/cmd_vel']
        (output / ('rqt_' + kind + '_state.json')).write_text(
            json.dumps({'node': rospy.get_name(), 'system_state': state,
                        'rendering': 'Real rqt plugin widget, Qt offscreen host'}, indent=2),
            encoding='utf-8')
        destination = output / ('rqt_' + kind + '.png')
        assert context.widget.grab().save(str(destination))
        print('PASS: live rqt_' + kind + ' widget captured: ' + str(destination))
    finally:
        plugin.shutdown_plugin()
        context.widget.close()
        rospy.signal_shutdown('Capture finished')


if __name__ == '__main__':
    main()
