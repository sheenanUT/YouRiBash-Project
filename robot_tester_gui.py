import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

from get_kinematics import *

# Robot Tester GUI class handles all of the GUI creation and interactions
class RobotTesterGUI(QMainWindow):
    def __init__(self):
        # Initializing main GUI window
        super().__init__()
        self.setWindowTitle('Robot Pick-and-Place Task Performance Tester')
        self.setGeometry(100, 100, 800, 600)

        # Initializing variables to track user selections
        self.simulation_robots = []
        self.simulation_tasks = []
        self.visualize_robot = None

        # Initializing main GUI widget
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        # Initializing different GUI pages as widgets
        self.simulationPage = QWidget()
        self.resultsPage = QWidget()

        # Initializing stacked widget that tracks different pages
        self.stacked_widget = QStackedWidget()
        self.main_layout.addWidget(self.stacked_widget)

        # Adding pages to stacked widget
        self.stacked_widget.addWidget(self.simulationPage)
        self.stacked_widget.addWidget(self.resultsPage)

        # Calling set up functions for each GUI page
        self.setup_simpage()
        self.setup_respage()

    def setup_simpage(self):
        # Set up function for first page of GUI that allows users to choose which robots and tasks are simulated
        layout = QVBoxLayout(self.simulationPage)

        # Create a horizontal section for the two input checkboxes
        checkbox_layout = QHBoxLayout()

        # Create a section for the robot selection checkbox
        robot_section = QVBoxLayout()
        robot_label = QLabel('Select Robots to Run the Simulation with:')
        robot_section.addWidget(robot_label)
        
        # Make the robot selection checkbox scrollable 
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        robot_scroll_widget = QWidget()
        robot_scroll_layout = QVBoxLayout(robot_scroll_widget)

        # Create a checkbox for each robot in the list
        self.robot_checkboxes = []
        robot_names = ['R1', 'R2', 'R3','R4', 'R5', 'R6','R7', 'R8', 'R9','R10', 'R11', 'R12','R13','R14','R15','R16','R17','R18','R19','R20','R21','R22','R23', 'R24', 'R25', 'R26']
        for name in robot_names:
            checkbox = QCheckBox(name)
            checkbox.stateChanged.connect(self.update_simulation_robots) # Update the variable that tracks which robots are selected
            robot_scroll_layout.addWidget(checkbox)
            self.robot_checkboxes.append(checkbox)
        
        # Set the scrollable area as the robot selection checkboxes and add it to robot selection section
        scroll_area.setWidget(robot_scroll_widget)
        robot_section.addWidget(scroll_area)

        # Add the robot selection checkbox section on the left side of the horizontal section for the two input checkboxes
        checkbox_layout.addLayout(robot_section, stretch=3)

        # Create a section for the task selection checkbox that is centered vertically
        task_section = QWidget()
        task_layout = QVBoxLayout(task_section)
        task_layout.setAlignment(Qt.AlignCenter)
        task_label = QLabel('Select Tasks to Simulate with the Robots:')
        task_layout.addWidget(task_label)

        # Create a checkbox for each task option
        self.tasks_checkboxes = []
        task_names = ['Basic', 'Obstacle', 'Elevation', 'Obstacle + Elevation']
        for name in task_names:
            checkbox = QCheckBox(name)
            checkbox.stateChanged.connect(self.update_simulation_tasks) # Update the variable that tracks which tasks are selected
            task_layout.addWidget(checkbox)
            self.tasks_checkboxes.append(checkbox)
        
        # Add the task selection checkbox section on the right side of the horizontal section for the two input checkboxes
        checkbox_layout.addWidget(task_section, stretch=1)

        # Add the horizontal section for the two input checkboxes to the main page layout
        layout.addLayout(checkbox_layout)

        # Create a run simulation button at the bottom of the GUI page that is initially hidden until one of each checkbox is selected
        self.run_simulation_button = QPushButton("Run Headless Simulation")
        self.run_simulation_button.setVisible(False)
        self.run_simulation_button.clicked.connect(self.run_simulation)
        layout.addWidget(self.run_simulation_button)
    
    def setup_respage(self):
        # Set up function for the second page of the GUI that allows users to view the results of the haedless simulation and pick one robot to visualize in Gazebo
        layout = QVBoxLayout(self.resultsPage)

        # Create a table at the top of the page that displays the headless simulation results
        layout.addWidget(QLabel('Pick-and-Place Simulation Results:'))
        self.results_table = QTableWidget(0,4)
        self.results_table.setHorizontalHeaderLabels(['Robot', 'Task', 'Time (s)', 'Total Distance (m)'])
        self.results_table.setColumnWidth(0, 150)
        layout.addWidget(self.results_table, stretch=3)

        # Create a section below the table for the button that selects a robot to visualize in Gazebo
        selection_section = QVBoxLayout()
        selection_section.addWidget(QLabel('Select a Robot to Simulate and Visualize with Gazebo'))

        # Make the robot visualization button section scrollable
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        robot_scroll_widget = QWidget()
        robot_scroll_layout = QVBoxLayout(robot_scroll_widget)

        # Create the buttons for each robot to be selected to visualize and simulate in Gazebo
        self.robot_radio_buttons = QButtonGroup()
        self.radio_buttons = []
        robot_names = ['R1', 'R2', 'R3','R4', 'R5', 'R6','R7', 'R8', 'R9','R10', 'R11', 'R12']
        for name in robot_names:
            radio_button = QRadioButton(name)
            radio_button.toggled.connect(self.update_visualize_robot) # Update the variable that tracks which robot is selected
            self.radio_buttons.append(radio_button)
            self.robot_radio_buttons.addButton(radio_button)
            robot_scroll_layout.addWidget(radio_button)

        # Add the scrollable section to the robot visualization button section
        scroll_area.setWidget(robot_scroll_widget)
        selection_section.addWidget(scroll_area)

        # Add a button below the scrollable robot selection to run the complete simulation and open Gazebo
        self.full_simulation_button = QPushButton('Run Complete Simulation and Visualize in Gazebo')
        self.full_simulation_button.clicked.connect(self.run_full_simulation)
        selection_section.addWidget(self.full_simulation_button)

        # Add the robot selection button and full simulation button to the main layout
        layout.addLayout(selection_section, stretch=1)

    def update_simulation_robots(self):
        # Function that updates the list of selected robots for the headless simulation
        self.simulation_robots = [checkbox.text() for checkbox in self.robot_checkboxes if checkbox.isChecked()]
        self.check_conditions()
    
    def update_simulation_tasks(self):
        # Function that updates the list of selected tasks for the headless simulation
        self.simulation_tasks = [checkbox.text() for checkbox in self.tasks_checkboxes if checkbox.isChecked()]
        self.check_conditions()

    def update_visualize_robot(self):
        # Function that updates depending on if a robot has been selected to fully simulate and visualize in Gazebo
        selected_button = self.robot_radio_buttons.checkedButton()
        if selected_button:
            self.visualize_robot = selected_button.text()
        else:
            self.visualize_robot = None

    def check_conditions(self):
        # Function that checks if a checkbox in the robot selection and task selection has been selected so that the run simulation button can appear
        if self.simulation_robots and self.simulation_tasks:
            self.run_simulation_button.setVisible(True)
        else:
            self.run_simulation_button.setVisible(False)

    def run_simulation(self):
        # Run simulation function that calls Gazebo and runs the simulation for each robot and each task
        results = []
        for robot in self.simulation_robots:
            for task in self.simulation_tasks:
                sim_data = np.array([[1,1,1,4],[1,1,1,4]])
                results_data = get_kinematics(sim_data)
                results.append({'Robot': robot, 'Task': task, 'Time (s)': results_data[1], 'Total Distance (m)': results_data[0]})
                
        # Updates the results table on the second page of the GUI with the results from the headless simulation
        self.results_table.setRowCount(len(results))
        for row, result in enumerate(results):
            self.results_table.setItem(row, 0, QTableWidgetItem(result['Robot']))
            self.results_table.setItem(row, 1, QTableWidgetItem(result['Task']))
            self.results_table.setItem(row, 2, QTableWidgetItem(str(result['Time (s)'])))
            self.results_table.setItem(row, 3, QTableWidgetItem(str(result['Total Distance (m)'])))
        self.results_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        # Switches to the results page after the simulation is complete
        self.stacked_widget.setCurrentWidget(self.resultsPage)
    
    def run_full_simulation(self):
        # Run full simulation function that opens Gazebo to simulate and visualize the robot selected by the user
        
        # Checks to make sure that a robot has been selected by the user and prints which robot will be simulated and visualized in Gazebo
        if self.visualize_robot:
            print(f'Running complete simulation and visuaizing in Gazebo for {self.visualize_robot}')
        else:
            print('No robot selected for complete simulation')

# Main application
app = QApplication(sys.argv)
window = RobotTesterGUI()
window.show()
sys.exit(app.exec_())