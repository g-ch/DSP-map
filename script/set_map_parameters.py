'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

Copyright <2022> <Gang Chen>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


Author: Gang Chen

Date: 2021/12/16

Description: This script is a tool designed to quickly modify the parameters in the DSP-Dynamic Map. There are many parameters in the DSP-Dynamic Map. This tool presents a UI using performance and efficiency levels to calculate and set parameters automatically.

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
import sys

def changeALineInAnyFile(file, unique_keywords, new_line):
    file_data = ""
    line_changed = False
    with open(file, "r") as f:
        for line in f:
            if unique_keywords in line and not line_changed:
                line = new_line
                line_changed = True
            file_data += line
    with open(file, "w") as f:
        f.write(file_data)
    return line_changed


def readParameterinFile(file, unique_keywords_of_the_parameter_definition_line, type="int"):
    parameter_found = False
    with open(file, "r") as f:
        for line in f:
            if unique_keywords_of_the_parameter_definition_line in line:
                copy_line = line
                parameter_value_str = copy_line.split("\n")[0].split("//")[0].split(";")[0].split(" ")[-1]
                print(parameter_value_str)
                parameter_found = True

                if type == "int":
                    return int(parameter_value_str)
                elif type == "float" or type == "double":
                    return float(parameter_value_str)
                elif type == "string" or type == "str":
                    return parameter_value_str
                else:
                    print("Unknown type in Function readCurrentParameter()")
    return parameter_found


class Q_Window(QWidget):

    def __init__(self):
        super(Q_Window, self).__init__()
        self.initializeParameters()
        self.readCurrentParameters()
        self.initUI()

    def initializeParameters(self):
        # File Path
        self.cpp_file = "../src/map_sim_example.cpp"
        self.head_file_dynamic = "../include/dsp_dynamic.h"

        # Parameters to modify in the UI
        self.resolution = 0.15
        self.map_length_x = 0.15*66
        self.map_length_y = 0.15*66
        self.map_length_z = 0.15*40
        self.efficiency = 45
        self.performance = 75

        self.fov_angle_h = 84
        self.fov_angle_v = 48

        self.filter_max_cluster_point_num = 200
        self.filter_max_cluster_center_height = 1.5

        # Map parameters related to performance and efficiency
        self.pyramid_resolution = 3
        self.max_particle_density = 3000
        self.voxel_filter = 0.1
        self.max_particle_num_voxel = int(self.max_particle_density * (self.resolution*self.resolution*self.resolution))

    def readCurrentParameters(self):
        self.resolution = readParameterinFile(self.head_file_dynamic, "#define VOXEL_RESOLUTION", type="float")
        self.map_length_x = readParameterinFile(self.head_file_dynamic, "#define MAP_LENGTH_VOXEL_NUM", type="int")*self.resolution
        self.map_length_y = readParameterinFile(self.head_file_dynamic, "#define MAP_WIDTH_VOXEL_NUM", type="int")*self.resolution
        self.map_length_z = readParameterinFile(self.head_file_dynamic, "#define MAP_HEIGHT_VOXEL_NUM", type="int")*self.resolution
        self.fov_angle_h = readParameterinFile(self.head_file_dynamic, "const int half_fov_h", type="int")*2
        self.fov_angle_v = readParameterinFile(self.head_file_dynamic, "const int half_fov_v", type="int")*2
        self.filter_max_cluster_point_num = readParameterinFile(self.head_file_dynamic, "DYNAMIC_CLUSTER_MAX_POINT_NUM", type="int")
        self.filter_max_cluster_center_height = readParameterinFile(self.head_file_dynamic, "DYNAMIC_CLUSTER_MAX_CENTER_HEIGHT", type="float")

        self.pyramid_resolution = readParameterinFile(self.head_file_dynamic, "#define ANGLE_RESOLUTION", type="int")
        self.max_particle_num_voxel = readParameterinFile(self.head_file_dynamic, "#define MAX_PARTICLE_NUM_VOXEL", type="int")
        self.voxel_filter = readParameterinFile(self.cpp_file, "const float res", type="float")

        self.max_particle_density = int(self.max_particle_num_voxel / (self.resolution*self.resolution*self.resolution))

        self.fov_angle_h_last = self.fov_angle_h
        self.fov_angle_v_last = self.fov_angle_v
        self.pyramid_resolution_last = self.pyramid_resolution

        # Calculate performance
        self.mapParametersToPerformanceLevel()

    def initUI(self):
        global desktop_size
        window_size = [1280, 720]

        ''' Resolution slider '''
        self.sl_res = QSlider(Qt.Horizontal, self)
        self.sl_res.setMinimum(10)
        self.sl_res.setMaximum(30)
        self.sl_res.setSingleStep(5)
        self.sl_res.setValue(int(self.resolution*100))
        self.sl_res.valueChanged[int].connect(self.resValueChange)

        slide_size = [600,40]
        slider_res_position = [int(window_size[0]/2 - slide_size[0]/2)+100, int(window_size[1]/2 - slide_size[1]/2 - 300)]
        self.sl_res.setGeometry(slider_res_position[0], slider_res_position[1], slide_size[0], slide_size[1])
        self.sl_res.setFocusPolicy(Qt.NoFocus)

        self.label_res = QLabel("Map Resolution (m)", self)
        self.label_res.move(slider_res_position[0]-300, slider_res_position[1]+10)
        self.label_res.setToolTip("Resolution of the voxel subspace. Range=[0.1,0.3]")
        self.line_edit_res = QLineEdit(str(self.resolution), self)
        self.line_edit_res.move(slider_res_position[0]-130, slider_res_position[1]+10)
        self.line_edit_res.resize(100,30)
        self.line_edit_res.textChanged.connect(self.resTextValueChanged)
        self.line_edit_res.setToolTip("Resolution of the voxel subspace. Range=[0.1,0.3]")

        res_validator = QDoubleValidator()
        res_validator.setRange(0.1, 0.3)
        res_validator.setDecimals(2)
        self.line_edit_res.setValidator(res_validator)

        ''' Map size  '''
        ### x
        self.sl_map_size_x = QSlider(Qt.Horizontal, self)
        self.sl_map_size_y = QSlider(Qt.Horizontal, self)
        self.sl_map_size_z = QSlider(Qt.Horizontal, self)

        self.sl_map_size_x.setMinimum(20)
        self.sl_map_size_x.setMaximum(200)
        self.sl_map_size_x.setSingleStep(int(self.resolution*100))
        self.sl_map_size_x.setValue(int(self.map_length_x*10))
        self.sl_map_size_x.valueChanged.connect(self.mapSizeChanged)
        sl_map_size_x_position = [int(window_size[0]/2 - slide_size[0]/2)+100, int(window_size[1]/2 - slide_size[1]/2 - 250)]
        self.sl_map_size_x.setGeometry(sl_map_size_x_position[0], sl_map_size_x_position[1], slide_size[0], slide_size[1])
        self.sl_map_size_x.setFocusPolicy(Qt.NoFocus)

        self.label_map_size_x = QLabel("Map Length (m)", self)
        self.label_map_size_x.move(sl_map_size_x_position[0]-300, sl_map_size_x_position[1]+10)
        self.label_value_map_size_x = QLabel(str(self.map_length_x), self)
        self.label_value_map_size_x.move(sl_map_size_x_position[0]-130, sl_map_size_x_position[1]+10)
        self.label_value_map_size_x.resize(100,30)

        ### y
        self.sl_map_size_y.setMinimum(20)
        self.sl_map_size_y.setMaximum(200)
        self.sl_map_size_y.setSingleStep(int(self.resolution*100))
        self.sl_map_size_y.setValue(int(self.map_length_y*10))
        self.sl_map_size_y.valueChanged.connect(self.mapSizeChanged)
        sl_map_size_y_position = [int(window_size[0]/2 - slide_size[0]/2)+100, int(window_size[1]/2 - slide_size[1]/2 - 200)]
        self.sl_map_size_y.setGeometry(sl_map_size_y_position[0], sl_map_size_y_position[1], slide_size[0], slide_size[1])
        self.sl_map_size_y.setFocusPolicy(Qt.NoFocus)

        self.label_map_size_y = QLabel("Map Width (m)", self)
        self.label_map_size_y.move(sl_map_size_y_position[0]-300, sl_map_size_y_position[1]+10)
        self.label_value_map_size_y = QLabel(str(self.map_length_y), self)
        self.label_value_map_size_y.move(sl_map_size_y_position[0]-130, sl_map_size_y_position[1]+10)
        self.label_value_map_size_y.resize(100,30)

        ### z
        self.sl_map_size_z.setMinimum(20)
        self.sl_map_size_z.setMaximum(200)
        self.sl_map_size_z.setSingleStep(int(self.resolution*100))
        self.sl_map_size_z.setValue(int(self.map_length_z*10))
        self.sl_map_size_z.valueChanged.connect(self.mapSizeChanged)
        sl_map_size_z_position = [int(window_size[0]/2 - slide_size[0]/2)+100, int(window_size[1]/2 - slide_size[1]/2 - 150)]
        self.sl_map_size_z.setGeometry(sl_map_size_z_position[0], sl_map_size_z_position[1], slide_size[0], slide_size[1])
        self.sl_map_size_z.setFocusPolicy(Qt.NoFocus)

        self.label_map_size_z = QLabel("Map Height (m)", self)
        self.label_map_size_z.move(sl_map_size_z_position[0]-300, sl_map_size_z_position[1]+10)
        self.label_value_map_size_z = QLabel(str(self.map_length_z), self)
        self.label_value_map_size_z.move(sl_map_size_z_position[0]-130, sl_map_size_z_position[1]+10)
        self.label_value_map_size_z.resize(100,30)


        ''' Performance and Efficiency '''
        self.sl_performance = QSlider(Qt.Horizontal, self)
        self.sl_performance.setMinimum(20)
        self.sl_performance.setMaximum(100)
        self.sl_performance.setSingleStep(2)
        self.sl_performance.setValue(self.performance)
        self.sl_performance.valueChanged.connect(self.performanceValueChange)

        slider_performance_position = [int(window_size[0]/2 - slide_size[0]/2)+100, int(window_size[1]/2 - slide_size[1]/2 - 100)]
        self.sl_performance.setGeometry(slider_performance_position[0], slider_performance_position[1], slide_size[0], slide_size[1])
        self.sl_performance.setFocusPolicy(Qt.NoFocus)

        self.label_performance = QLabel("Performance", self)
        self.label_performance.move(slider_performance_position[0]-300, slider_performance_position[1]+10)
        self.label_performance = QLabel(str(self.performance), self)
        self.label_performance.move(slider_performance_position[0]-130, slider_performance_position[1]+10)
        self.label_performance.resize(100,30)

        self.sl_efficiency = QSlider(Qt.Horizontal, self)
        self.sl_efficiency.setMinimum(20)
        self.sl_efficiency.setMaximum(100)
        self.sl_efficiency.setSingleStep(2)
        self.sl_efficiency.setValue(self.efficiency)
        self.sl_efficiency.valueChanged.connect(self.efficiencyValueChange)

        slider_efficiency_position = [int(window_size[0]/2 - slide_size[0]/2)+100, int(window_size[1]/2 - slide_size[1]/2 - 50)]
        self.sl_efficiency.setGeometry(slider_efficiency_position[0], slider_efficiency_position[1], slide_size[0], slide_size[1])
        self.sl_efficiency.setFocusPolicy(Qt.NoFocus)

        self.label_efficiency = QLabel("Efficiency", self)
        self.label_efficiency.move(slider_efficiency_position[0]-300, slider_efficiency_position[1]+10)
        self.label_efficiency = QLabel(str(self.efficiency), self)
        self.label_efficiency.move(slider_efficiency_position[0]-130, slider_efficiency_position[1]+10)
        self.label_efficiency.resize(100,30)

        ''' FOV '''
        fov_edit_position = [int(window_size[0]/2 - slide_size[0]/2)+100, int(window_size[1]/2 - slide_size[1]/2)+30]

        self.label_fov_h = QLabel("FOV Horizontal Angle", self)
        self.label_fov_h.move(fov_edit_position[0]-300, fov_edit_position[1]+10)
        self.label_fov_h.setToolTip("Camera parameter. Degree. Range=[1,120]")
        self.line_edit_fov_h = QLineEdit(str(self.fov_angle_h), self)
        self.line_edit_fov_h.move(fov_edit_position[0]-100, fov_edit_position[1]+10)
        self.line_edit_fov_h.resize(100,30)
        self.line_edit_fov_h.textChanged.connect(self.fovHTextValueChanged)
        self.line_edit_fov_h.setToolTip("Camera parameter. Degree. Range=[1,120]")

        fov_h_validator = QIntValidator()
        fov_h_validator.setRange(1, 120)
        self.line_edit_fov_h.setValidator(fov_h_validator)

        self.label_fov_v = QLabel("FOV Vertical Angle", self)
        self.label_fov_v.move(fov_edit_position[0]+100, fov_edit_position[1]+10)
        self.label_fov_v.setToolTip("Camera parameter. Degree. Range=[1,90]")
        self.line_edit_fov_v = QLineEdit(str(self.fov_angle_v), self)
        self.line_edit_fov_v.move(fov_edit_position[0]+300, fov_edit_position[1]+10)
        self.line_edit_fov_v.resize(100,30)
        self.line_edit_fov_v.textChanged.connect(self.fovVTextValueChanged)
        self.line_edit_fov_v.setToolTip("Camera parameter. Degree. Range=[1,90]")

        fov_v_validator = QIntValidator()
        fov_v_validator.setRange(1, 90)
        self.line_edit_fov_h.setValidator(fov_v_validator)

        ''' Filter for initial velocity estimation '''
        filter_label_position = [int(window_size[0]/2 - slide_size[0]/2)+100, int(window_size[1]/2 - slide_size[1]/2)+130]
        self.label_filter = QLabel("*Optional: paramters of the filter for initial velocity estimation", self)
        self.label_filter.move(filter_label_position[0]-300, filter_label_position[1])

        filter_edit_position = [int(window_size[0]/2 - slide_size[0]/2)+120, int(window_size[1]/2 - slide_size[1]/2)+160]
        self.label_filter_max_number = QLabel("Max cluster point number", self)
        self.label_filter_max_number.move(filter_edit_position[0]-300, filter_edit_position[1]+10)
        self.line_filter_max_number = QLineEdit(str(self.filter_max_cluster_point_num), self)
        self.line_filter_max_number.move(filter_edit_position[0]-50, filter_edit_position[1]+10)
        self.line_filter_max_number.resize(100,30)
        self.line_filter_max_number.textChanged.connect(self.filterMaxNumberTextValueChanged)
        self.line_filter_max_number.setToolTip("The allowed maximum point number in a dynamic obstacle cluster.")

        filter_max_number_validator = QIntValidator()
        filter_max_number_validator.setRange(1, 10000)
        self.line_filter_max_number.setValidator(filter_max_number_validator)


        self.label_filter_max_height = QLabel("Max cluster center height", self)
        self.label_filter_max_height.move(filter_edit_position[0] + 100, filter_edit_position[1]+10)
        self.line_filter_max_height = QLineEdit(str(self.filter_max_cluster_center_height), self)
        self.line_filter_max_height.move(filter_edit_position[0]+350, filter_edit_position[1]+10)
        self.line_filter_max_height.resize(100,30)
        self.line_filter_max_height.textChanged.connect(self.filterCenterHeightTextValueChanged)
        self.line_filter_max_number.setToolTip("The allowed maximum center height of a dynamic obstacle cluster.")


        filter_max_height_validator = QDoubleValidator()
        filter_max_height_validator.setRange(0.1, 100.0)
        filter_max_height_validator.setDecimals(2)
        self.line_filter_max_height.setValidator(filter_max_height_validator)

        ''' Reset and Confirm button '''
        button_position = [int(window_size[0]/2 - slide_size[0]/2)-100, int(window_size[1]/2 - slide_size[1]/2)+250]
        self.reset_button = QPushButton("Reset to default", self)
        self.reset_button.move(button_position[0], button_position[1])
        self.reset_button.resize(250,80)
        self.reset_button.clicked.connect(self.resetButtonClicked);

        self.save_button = QPushButton("Save", self)
        self.save_button.move(button_position[0]+500, button_position[1])
        self.save_button.resize(250,80)
        self.save_button.clicked.connect(self.saveButtonClicked);

        ''' Main Window'''
        self.setGeometry(int(desktop_size[0]/2 - window_size[0]/2), int(desktop_size[1]/2 - window_size[1]/2), window_size[0], window_size[1])
        self.setWindowTitle('MapParameterQuickTuner')
        self.show()

    def resValueChange(self, value):
        self.resolution = value/100
        self.line_edit_res.setText(str(value/100))
        self.sl_map_size_x.setSingleStep(int(self.resolution*100))
        self.sl_map_size_y.setSingleStep(int(self.resolution*100))
        self.sl_map_size_z.setSingleStep(int(self.resolution*100))

    def resTextValueChanged(self):
        self.resolution = float(self.line_edit_res.text())
        self.sl_res.setValue(int(self.resolution*100))
        self.sl_map_size_x.setSingleStep(int(self.resolution*100))
        self.sl_map_size_y.setSingleStep(int(self.resolution*100))
        self.sl_map_size_z.setSingleStep(int(self.resolution*100))

    def mapSizeChanged(self):
        self.map_length_x = self.sl_map_size_x.value()/10
        self.map_length_y = self.sl_map_size_y.value()/10
        self.map_length_z = self.sl_map_size_z.value()/10

        self.label_value_map_size_x.setText(str(self.map_length_x))
        self.label_value_map_size_y.setText(str(self.map_length_y))
        self.label_value_map_size_z.setText(str(self.map_length_z))

    def efficiencyValueChange(self):
        self.efficiency = self.sl_efficiency.value()
        self.label_efficiency.setText(str(self.efficiency))
        self.performance = 120 - self.sl_efficiency.value()
        self.label_performance.setText(str(self.performance))
        self.sl_performance.setValue(self.performance)

    def performanceValueChange(self):
        self.performance = self.sl_performance.value()
        self.label_performance.setText(str(self.performance))
        self.efficiency = 120 - self.performance
        self.label_efficiency.setText(str(self.efficiency))
        self.sl_efficiency.setValue(self.efficiency)

    def fovHTextValueChanged(self):
        self.fov_angle_h = int(self.line_edit_fov_h.text())

    def fovVTextValueChanged(self):
        self.fov_angle_v = int(self.line_edit_fov_v.text())

    def filterMaxNumberTextValueChanged(self):
        self.filter_max_cluster_point_num = int(self.line_filter_max_number.text())

    def filterCenterHeightTextValueChanged(self):
        self.filter_max_cluster_center_height = float(self.line_filter_max_height.text())

    def resetButtonClicked(self):
        A = QMessageBox.question(self,'Confirm','Are you sure to reset the parameters？',QMessageBox.Yes | QMessageBox.No)
        if A == QMessageBox.Yes:
            self.initializeParameters()
            self.updateUI()

    def saveButtonClicked(self):
        if self.resolution > 0.3 or self.resolution < 0.1:
            msg = QMessageBox.warning(self, "Warning", "Map resolution should be in the range [0.1, 0.3]", buttons=QMessageBox.Ok)
            return

        if self.fov_angle_h > 120 or self.fov_angle_h < 0:
            msg = QMessageBox.warning(self, "Warning", "Horizontal FOV angle should be in the range [0, 120]", buttons=QMessageBox.Ok)
            return

        if self.fov_angle_v > 90 or self.fov_angle_v < 0:
            msg = QMessageBox.warning(self, "Warning", "Vertical FOV angle should be in the range [0, 90]", buttons=QMessageBox.Ok)
            return

        ''' Calculate Specific Map parameters and save '''
        A = QMessageBox.question(self,'Confirm','Are you sure to save the parameter？',QMessageBox.Yes | QMessageBox.No)
        if A == QMessageBox.Yes:
            self.performanceLevelToMapParameters()
            self.max_particle_num_voxel = int(self.max_particle_density * (self.resolution*self.resolution*self.resolution))
            # Set a minimum particle number in a voxel
            if self.max_particle_num_voxel < 5:
                self.max_particle_num_voxel = 5

            if not changeALineInAnyFile(self.head_file_dynamic, "#define VOXEL_RESOLUTION", "#define VOXEL_RESOLUTION " + str(self.resolution) + "\n"):
                QMessageBox.critical(self, "Error", "Cannot find VOXEL_RESOLUTION in " + self.head_file_dynamic +"!", buttons=QMessageBox.Ok)
                return

            if not changeALineInAnyFile(self.head_file_dynamic, "#define MAP_LENGTH_VOXEL_NUM", "#define MAP_LENGTH_VOXEL_NUM " + str(int(self.map_length_x/self.resolution)) + "\n"):
                QMessageBox.critical(self, "Error", "Cannot find MAP_LENGTH_VOXEL_NUM in " + self.head_file_dynamic +"!", buttons=QMessageBox.Ok)
                return

            if not changeALineInAnyFile(self.head_file_dynamic, "#define MAP_WIDTH_VOXEL_NUM", "#define MAP_WIDTH_VOXEL_NUM " + str(int(self.map_length_y/self.resolution)) + "\n"):
                QMessageBox.critical(self, "Error", "Cannot find MAP_WIDTH_VOXEL_NUM in " + self.head_file_dynamic +"!", buttons=QMessageBox.Ok)
                return

            if not changeALineInAnyFile(self.head_file_dynamic, "#define MAP_HEIGHT_VOXEL_NUM", "#define MAP_HEIGHT_VOXEL_NUM " + str(int(self.map_length_z/self.resolution)) + "\n"):
                QMessageBox.critical(self, "Error", "Cannot find MAP_HEIGHT_VOXEL_NUM in " + self.head_file_dynamic +"!", buttons=QMessageBox.Ok)
                return

            if not changeALineInAnyFile(self.head_file_dynamic, "#define DYNAMIC_CLUSTER_MAX_POINT_NUM", "#define DYNAMIC_CLUSTER_MAX_POINT_NUM " + str(self.filter_max_cluster_point_num) + "\n"):
                QMessageBox.critical(self, "Error", "Cannot find DYNAMIC_CLUSTER_MAX_POINT_NUM in " + self.head_file_dynamic +"!", buttons=QMessageBox.Ok)
                return

            if not changeALineInAnyFile(self.head_file_dynamic, "#define DYNAMIC_CLUSTER_MAX_CENTER_HEIGHT", "#define DYNAMIC_CLUSTER_MAX_CENTER_HEIGHT " + str(self.filter_max_cluster_center_height) + "\n"):
                QMessageBox.critical(self, "Error", "Cannot find DYNAMIC_CLUSTER_MAX_CENTER_HEIGHT in " + self.head_file_dynamic +"!", buttons=QMessageBox.Ok)
                return

            if not changeALineInAnyFile(self.head_file_dynamic, "#define ANGLE_RESOLUTION", "#define ANGLE_RESOLUTION " + str(self.pyramid_resolution) + "\n"):
                QMessageBox.critical(self, "Error", "Cannot find ANGLE_RESOLUTION in " + self.head_file_dynamic +"!", buttons=QMessageBox.Ok)
                return

            if not changeALineInAnyFile(self.head_file_dynamic, "#define MAX_PARTICLE_NUM_VOXEL", "#define MAX_PARTICLE_NUM_VOXEL " + str(self.max_particle_num_voxel) + "\n"):
                QMessageBox.critical(self, "Error", "Cannot find MAX_PARTICLE_NUM_VOXEL in " + self.head_file_dynamic +"!", buttons=QMessageBox.Ok)
                return

            if not changeALineInAnyFile(self.cpp_file, "const float res", "const float res = " + str(self.voxel_filter) + ";\n"):
                QMessageBox.critical(self, "Error", "Cannot find float res in " + self.cpp_file +"!", buttons=QMessageBox.Ok)
                return

            ''' Change threshold and new born particle number '''
            occupancy_threshold = 0.2
            if self.resolution > 0.18:
                occupancy_threshold = 0.5
            if self.resolution > 0.28:
                occupancy_threshold = 0.6

            if not changeALineInAnyFile(self.cpp_file, "my_map.getOccupancyMapWithFutureStatus(occupied_num",
                                        "      my_map.getOccupancyMapWithFutureStatus(occupied_num, cloud_to_publish, &future_status[0][0], "
                                        + str(occupancy_threshold) + ");\n"):
                QMessageBox.critical(self, "Error", "Cannot find Function getOccupancyMapWithFutureStatus in " + self.cpp_file +"!", buttons=QMessageBox.Ok)
                return

            ''' Change FOV '''
            if self.fov_angle_h != self.fov_angle_h_last or self.pyramid_resolution != self.pyramid_resolution_last:
                half_fov_angle_h = int((self.fov_angle_h-self.pyramid_resolution) / 2 / self.pyramid_resolution)*self.pyramid_resolution # Abort the measurement close to fov's edge.
                if not changeALineInAnyFile(self.head_file_dynamic, "const int half_fov_h", "const int half_fov_h = " + str(half_fov_angle_h) + ";\n"):
                    QMessageBox.critical(self, "Error", "Cannot find Parameter half_fov_h in " + self.head_file_dynamic +"!", buttons=QMessageBox.Ok)
                    return

            if self.fov_angle_v != self.fov_angle_v_last or self.pyramid_resolution != self.pyramid_resolution_last:
                half_fov_angle_v = int((self.fov_angle_v-self.pyramid_resolution) / 2 / self.pyramid_resolution)*self.pyramid_resolution # Abort the measurement close to fov's edge.
                if not changeALineInAnyFile(self.head_file_dynamic, "const int half_fov_v", "const int half_fov_v = " + str(half_fov_angle_v) + ";\n"):
                    QMessageBox.critical(self, "Error", "Cannot find Parameter half_fov_v in " + self.head_file_dynamic +"!", buttons=QMessageBox.Ok)
                    return

            QMessageBox.information(self, "Note", "Parameters modified successfully! Please recompile the map using catkin_make or catkin build.", buttons=QMessageBox.Ok)

            self.readCurrentParameters()
            self.updateUI()

    def performanceLevelToMapParameters(self):
        if self.performance < 35:
            self.pyramid_resolution = 1
            self.voxel_filter = 0.2
            self.max_particle_density = int((3000-1000)*(self.performance-20)/15 + 1000)
        elif self.performance < 50:
            self.pyramid_resolution = 1
            self.voxel_filter = 0.15
            self.max_particle_density = int((3000-2000)*(self.performance-35)/15 + 2000)
        elif self.performance < 70:
            self.pyramid_resolution = 3
            self.voxel_filter = 0.15
            self.max_particle_density = int((3000-2000)*(self.performance-50)/20 + 2000)
        else:
            self.pyramid_resolution = 3
            self.voxel_filter = 0.1
            self.max_particle_density = int((6000-2500)*(self.performance-70)/30 + 2500)

    def mapParametersToPerformanceLevel(self):
        if self.pyramid_resolution < 2:
            if self.voxel_filter > 0.18: # 20 < performance < 35
                self.performance = int((self.max_particle_density - 1000)/(3000-1000)*15+20)
            else: # 35 < performance < 50
                self.performance = int((self.max_particle_density - 2000)/(3000-2000)*15+35)
        else:
            if self.voxel_filter > 0.12: # 50 < performance < 70
                self.performance = int((self.max_particle_density - 2000)/(3000-2000)*20+50)
            else: # 70 < performance < 100
                self.performance = int((self.max_particle_density - 2500)/(6000-2500)*30+70)
        if self.performance > 100:
            self.performance = 100

        self.efficiency = 120-self.performance

    def updateUI(self):
        self.sl_res.setValue(int(self.resolution*100))
        self.sl_map_size_x.setValue(int(self.map_length_x*10))
        self.sl_map_size_y.setValue(int(self.map_length_y*10))
        self.sl_map_size_z.setValue(int(self.map_length_z*10))
        self.sl_efficiency.setValue(self.efficiency)
        self.sl_performance.setValue(self.performance)
        self.line_edit_fov_h.setText(str(self.fov_angle_h))
        self.line_edit_fov_v.setText(str(self.fov_angle_v))
        self.line_filter_max_number.setText(str(self.filter_max_cluster_point_num))
        self.line_filter_max_height.setText(str(self.filter_max_cluster_center_height))

if __name__ == '__main__':
    desktop_size=[0,0]
    app = QApplication(sys.argv)
    desktop_size[0] = app.desktop().width()
    desktop_size[1] = app.desktop().height()

    ex = Q_Window()
    sys.exit(app.exec_())