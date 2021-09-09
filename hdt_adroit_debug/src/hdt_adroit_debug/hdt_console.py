# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division
import os
import rospkg

from geometry_msgs.msg import Twist
import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import QShortcut, QWidget
from rqt_gui_py.plugin import Plugin


class HDTconsole(Plugin):

  slider_factor = 1000.0

  def __init__(self, context):
    super(HDTconsole, self).__init__(context)
    self.setObjectName('HDTconsole')

    self._publisher = None

    self._widget = QWidget()
    rp = rospkg.RosPack()
    ui_file = os.path.join(
        rp.get_path('hdt_adroit_debug'), 'resource', 'HDTconsole.ui')
    loadUi(ui_file, self._widget)
    self._widget.setObjectName('HDTconsoleUi')
    if context.serial_number() > 1:
        self._widget.setWindowTitle(
            self._widget.windowTitle() + (' (%d)' % context.serial_number()))
    context.add_widget(self._widget)

    # get startup CAN ID an command rate
    can_id = self._widget.can_id_input.value()
    self._widget.can_id_input.valueChanged.connect(self._on_can_id_input_changed)
        
    cmd_rate = self._widget.cmd_rate_input.value()
    self._widget.cmd_rate_input.valueChanged.connect(
      self._on_cmd_rate_input_changed)
    
    cmd_enabled = self._widget.send_command_button.isChecked()
    self._widget.send_command_button.clicked.connect(
      self._on_send_command_button_changed)
    self._widget.send_command_button.setStyleSheet("""
        QPushButton::checked{background:rgb(255, 0, 0);}
    """)
    # setup sliders
    self._widget.position_slider.valueChanged.connect(
        self._on_position_slider_changed)


    self._widget.increase_x_linear_push_button.pressed.connect(
        self._on_strong_increase_x_linear_pressed)
    self._widget.reset_x_linear_push_button.pressed.connect(
        self._on_reset_x_linear_pressed)
    self._widget.decrease_x_linear_push_button.pressed.connect(
        self._on_strong_decrease_x_linear_pressed)


    self._widget.max_x_linear_double_spin_box.valueChanged.connect(
        self._on_max_x_linear_changed)
    self._widget.min_x_linear_double_spin_box.valueChanged.connect(
        self._on_min_x_linear_changed)


    self.shortcut_w = QShortcut(QKeySequence(Qt.Key_W), self._widget)
    self.shortcut_w.setContext(Qt.ApplicationShortcut)
    self.shortcut_w.activated.connect(self._on_increase_x_linear_pressed)
    self.shortcut_x = QShortcut(QKeySequence(Qt.Key_X), self._widget)
    self.shortcut_x.setContext(Qt.ApplicationShortcut)
    self.shortcut_x.activated.connect(self._on_reset_x_linear_pressed)
    self.shortcut_s = QShortcut(QKeySequence(Qt.Key_S), self._widget)
    self.shortcut_s.setContext(Qt.ApplicationShortcut)
    self.shortcut_s.activated.connect(self._on_decrease_x_linear_pressed)


    # timer to consecutively send twist messages
    self._update_parameter_timer = QTimer(self)
    self._update_parameter_timer.timeout.connect(
        self._on_parameter_changed)
    self._update_parameter_timer.start(100)
    self.zero_cmd_sent = False

  @Slot(str)
  def _on_can_id_input_changed(self):
    can_id = self._widget.can_id_input.value()
    
  def _on_cmd_rate_input_changed(self):
    cmd_rate = self._widget.cmd_rate_input.value()
    
  def _on_send_command_button_changed(self):
    cmd_enabled = self._widget.send_command_button.isChecked()
    
    
  def _on_stop_pressed(self):
      # If the current value of sliders is zero directly send stop twist msg
      if self._widget.position_slider.value() == 0:
          self.zero_cmd_sent = False
          self._on_parameter_changed()
      else:
          self._widget.position_slider.setValue(0)

  def _on_position_slider_changed(self):
    # commanded position slider changed
    rospy.loginfo("position slider is {} in [{},{}]".format(self._widget.position_slider.value(),self._widget.position_slider.maximum(),self._widget.position_slider.minimum()))
    self._widget.position_label.setText(
        '%0.2f rad' % (self._widget.position_slider.value() / HDTconsole.slider_factor))
    self._on_parameter_changed()

  def _on_increase_x_linear_pressed(self):
      self._widget.position_slider.setValue(
          self._widget.position_slider.value() + self._widget.position_slider.singleStep())

  def _on_reset_x_linear_pressed(self):
      self._widget.position_slider.setValue(0)

  def _on_decrease_x_linear_pressed(self):
      self._widget.position_slider.setValue(
          self._widget.position_slider.value() - self._widget.position_slider.singleStep())

  def _on_max_x_linear_changed(self, value):
      self._widget.position_slider.setMaximum(
          value * HDTconsole.slider_factor)

  def _on_min_x_linear_changed(self, value):
      self._widget.position_slider.setMinimum(
          value * HDTconsole.slider_factor)

  def _on_strong_increase_x_linear_pressed(self):
      self._widget.position_slider.setValue(
          self._widget.position_slider.value() + self._widget.position_slider.pageStep())

  def _on_strong_decrease_x_linear_pressed(self):
      self._widget.position_slider.setValue(
          self._widget.position_slider.value() - self._widget.position_slider.pageStep())

  def _on_parameter_changed(self):
      self._send_twist(
          self._widget.position_slider.value() / HDTconsole.slider_factor,
          0/ HDTconsole.slider_factor)

  def _send_twist(self, x_linear, z_angular):
      if self._publisher is None:
          return
      twist = Twist()
      twist.linear.x = x_linear
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0
      twist.angular.y = 0
      twist.angular.z = 0

      # Only send the zero command once so other devices can take control
      if x_linear == 0:
          if not self.zero_cmd_sent:
              self.zero_cmd_sent = True
              self._publisher.publish(twist)
      else:
          self.zero_cmd_sent = False
          self._publisher.publish(twist)

  def _unregister_publisher(self):
      if self._publisher is not None:
          self._publisher.unregister()
          self._publisher = None

  def shutdown_plugin(self):
      self._update_parameter_timer.stop()
      self._unregister_publisher()

  def save_settings(self, plugin_settings, instance_settings):
      #instance_settings.set_value(
      #    'topic', self._widget.topic_line_edit.text())
      instance_settings.set_value(
          'vx_max', self._widget.max_x_linear_double_spin_box.value())
      instance_settings.set_value(
          'vx_min', self._widget.min_x_linear_double_spin_box.value())


  def restore_settings(self, plugin_settings, instance_settings):
      #value = instance_settings.value('topic', '/cmd_velocity')
      #value = rospy.get_param('~default_topic', value)
      #self._widget.topic_line_edit.setText(value)

      value = self._widget.max_x_linear_double_spin_box.value()
      value = instance_settings.value('vx_max', value)
      value = rospy.get_param('~default_vx_max', value)
      self._widget.max_x_linear_double_spin_box.setValue(float(value))

      value = self._widget.min_x_linear_double_spin_box.value()
      value = instance_settings.value('vx_min', value)
      value = rospy.get_param('~default_vx_min', value)
      self._widget.min_x_linear_double_spin_box.setValue(float(value))

